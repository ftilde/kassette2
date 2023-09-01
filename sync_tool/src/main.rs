mod media_definition;

use clap::Parser;
use progressing::Baring;
use rubato::{InterpolationParameters, InterpolationType, Resampler, SincFixedIn, WindowFunction};
use std::{
    collections::VecDeque,
    fs::File,
    path::{Path, PathBuf},
};
use symphonia::core::{
    audio::Signal,
    codecs::{DecoderOptions, CODEC_TYPE_NULL},
    errors::Error,
    formats::FormatOptions,
    io::MediaSourceStream,
    meta::MetadataOptions,
    probe::Hint,
};
use vorbis_rs::VorbisEncoderBuilder;

#[derive(Debug, Copy, Clone, clap::ValueEnum)]
enum Version {
    V1,
    V2,
}

#[derive(Parser, Debug)]
#[command()]
struct Args {
    #[arg()]
    source: PathBuf,

    #[arg()]
    destination: PathBuf,

    #[arg(short = 'V')]
    version_overwrite: Option<Version>,
}

fn transcode_v1(source: &Path, destination: &Path) {
    let src = File::open(source).expect("failed to open media");

    let destination_tmp = destination.with_file_name({
        let mut filename = destination.file_name().unwrap().to_owned();
        filename.push(".part");
        filename
    });

    let out_file = File::create(&destination_tmp).unwrap();
    let mut out_file = std::io::BufWriter::new(out_file);
    {
        // Create the media source stream.
        let mss = MediaSourceStream::new(Box::new(src), Default::default());

        // Create a probe hint using the file's extension. [Optional]
        let mut hint = Hint::new();
        if let Some(ext) = source.extension() {
            hint.with_extension(&ext.to_string_lossy());
        }

        // Use the default options for metadata and format readers.
        let meta_opts: MetadataOptions = Default::default();
        let fmt_opts: FormatOptions = Default::default();

        // Probe the media source.
        let probed = symphonia::default::get_probe()
            .format(&hint, mss, &fmt_opts, &meta_opts)
            .expect("unsupported format");

        // Get the instantiated format reader.
        let mut format = probed.format;

        // Find the first audio track with a known (decodeable) codec.
        let track = format
            .tracks()
            .iter()
            .find(|t| t.codec_params.codec != CODEC_TYPE_NULL)
            .expect("no supported audio tracks");

        let input_sample_rate = track.codec_params.sample_rate.unwrap();
        let len = track.codec_params.n_frames;
        let len_estimated = len.unwrap();
        let channels = track.codec_params.channels.unwrap().count();

        let mut encoder = VorbisEncoderBuilder::new(
            std::num::NonZeroU32::new(input_sample_rate).unwrap(),
            std::num::NonZeroU8::new(channels as _).unwrap(),
            &mut out_file,
        )
        .unwrap()
        .build()
        .unwrap();

        let target_sample_rate = 44100;

        let frames_per_batch = 1024;

        let params = InterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: InterpolationType::Linear,
            oversampling_factor: 256,
            window: WindowFunction::BlackmanHarris2,
        };
        let factor = target_sample_rate as f64 / input_sample_rate as f64;
        let mut resampler =
            SincFixedIn::<f32>::new(factor, 1.0 / factor, params, frames_per_batch, channels)
                .unwrap();

        // Use the default options for the decoder.
        let dec_opts: DecoderOptions = Default::default();

        // Create a decoder for the track.
        let mut decoder = symphonia::default::get_codecs()
            .make(&track.codec_params, &dec_opts)
            .expect("unsupported codec");

        // Store the track identifier, it will be used to filter packets.
        let track_id = track.id;

        let mut frame_count = 0;

        let mut out_bufs: Vec<VecDeque<f32>> =
            std::iter::repeat(VecDeque::with_capacity(2 * frames_per_batch))
                .take(channels)
                .collect();

        let mut resample_and_output = |batches: &[Vec<f32>]| {
            let resampled = resampler.process(batches, None).unwrap();

            encoder.encode_audio_block(resampled).unwrap();
        };

        let mut progress_bar =
            progressing::mapping::Bar::with_range(0, len_estimated as i64).timed();

        // The decode loop.
        loop {
            // Get the next packet from the media format.
            let packet = match format.next_packet() {
                Ok(packet) => packet,
                Err(Error::IoError(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    // Done
                    break;
                }
                Err(err) => {
                    // A unrecoverable error occured, halt decoding.
                    panic!("{}", err);
                }
            };

            // Consume any new metadata that has been read since the last packet.
            while !format.metadata().is_latest() {
                // Pop the old head of the metadata queue.
                format.metadata().pop();

                // Consume the new metadata at the head of the metadata queue.
            }

            // If the packet does not belong to the selected track, skip over it.
            if packet.track_id() != track_id {
                continue;
            }

            // Decode the packet into audio samples.
            match decoder.decode(&packet) {
                Ok(decoded) => {
                    let num_channels = decoded.spec().channels.count();
                    let mut buf = decoded.make_equivalent::<f32>();

                    decoded.convert(&mut buf);
                    match num_channels {
                        1 => {
                            out_bufs[0].extend(buf.chan(0));
                        }
                        2 => {
                            out_bufs[0].extend(buf.chan(0));
                            out_bufs[1].extend(buf.chan(1));
                        }
                        o => panic!("Unsupported number of channels: {}", o),
                    }
                    if out_bufs[0].len() > frames_per_batch {
                        let batches = out_bufs
                            .iter_mut()
                            .map(|b| b.drain(..frames_per_batch).collect())
                            .collect::<Vec<Vec<_>>>();
                        resample_and_output(&batches);
                        frame_count += batches[0].len();
                        progress_bar.set(frame_count as i64);
                        if progress_bar.has_progressed_significantly() {
                            print!("\r{}", progress_bar);
                        }
                    }

                    //out_file.write_all(out_buf).unwrap();
                    //println!("Written {} samples", out_buf.len());
                }
                Err(Error::IoError(_)) => {
                    // The packet failed to decode due to an IO error, skip the packet.
                    continue;
                }
                Err(Error::DecodeError(_)) => {
                    // The packet failed to decode due to invalid data, skip the packet.
                    continue;
                }
                Err(err) => {
                    // An unrecoverable error occured, halt decoding.
                    panic!("{:?}", err);
                }
            }
        }
        let batches = out_bufs
            .into_iter()
            .map(|mut b| {
                b.drain(..)
                    .chain(std::iter::repeat(0.0))
                    .take(frames_per_batch)
                    .collect()
            })
            .collect::<Vec<Vec<_>>>();
        resample_and_output(&batches);

        encoder.finish().unwrap();
    }

    std::fs::rename(destination_tmp, destination).unwrap();

    println!(" Done");
}

fn transcode_v2(source: &Path, destination: &Path) {
    // Open the media source.
    let src = File::open(source).expect("failed to open media");

    let destination_tmp = destination.with_file_name({
        let mut filename = destination.file_name().unwrap().to_owned();
        filename.push(".part");
        filename
    });
    let out_file = File::create(&destination_tmp).unwrap();
    let out_file = std::io::BufWriter::new(out_file);

    let target_sample_rate = config::SAMPLE_RATE;

    let bits = config::BITS_PER_SAMPLE as u32;
    let format_bits = config::FORMAT_BITS_PER_SAMPLE as u32;
    assert!(format_bits >= bits);

    {
        // Create the media source stream.
        let mss = MediaSourceStream::new(Box::new(src), Default::default());

        // Create a probe hint using the file's extension. [Optional]
        let mut hint = Hint::new();
        if let Some(ext) = source.extension() {
            hint.with_extension(&ext.to_string_lossy());
        }

        // Use the default options for metadata and format readers.
        let meta_opts: MetadataOptions = Default::default();
        let fmt_opts: FormatOptions = Default::default();

        // Probe the media source.
        let probed = symphonia::default::get_probe()
            .format(&hint, mss, &fmt_opts, &meta_opts)
            .expect("unsupported format");

        // Get the instantiated format reader.
        let mut format = probed.format;

        // Find the first audio track with a known (decodeable) codec.
        let track = format
            .tracks()
            .iter()
            .find(|t| t.codec_params.codec != CODEC_TYPE_NULL)
            .expect("no supported audio tracks");

        let input_sample_rate = track.codec_params.sample_rate.unwrap();
        let len = track.codec_params.n_frames;
        let len_estimated = len.unwrap();

        //let target_sample_rate = input_sample_rate;

        const OUT_CHANNELS: u32 = 1;
        let mut frame_buffer = [0u8; embedded_qoa::max_frame_size(OUT_CHANNELS)];
        let mut sample_buffer = [0i16; embedded_qoa::max_sample_buffer_len(OUT_CHANNELS)];
        let mut writer = embedded_qoa::Writer::new(
            out_file,
            OUT_CHANNELS,
            target_sample_rate,
            Some(len_estimated as _),
            &mut frame_buffer,
            &mut sample_buffer,
        )
        .unwrap();

        //let mut resampler = Resampler::new(input_sample_rate as _, target_sample_rate as _);

        let frames_per_batch = 1024;

        let params = InterpolationParameters {
            sinc_len: 256,
            f_cutoff: 0.95,
            interpolation: InterpolationType::Linear,
            oversampling_factor: 256,
            window: WindowFunction::BlackmanHarris2,
        };
        let factor = target_sample_rate as f64 / input_sample_rate as f64;
        let mut resampler =
            SincFixedIn::<f32>::new(factor, 1.0 / factor, params, frames_per_batch, 1).unwrap();

        // Use the default options for the decoder.
        let dec_opts: DecoderOptions = Default::default();

        // Create a decoder for the track.
        let mut decoder = symphonia::default::get_codecs()
            .make(&track.codec_params, &dec_opts)
            .expect("unsupported codec");

        // Store the track identifier, it will be used to filter packets.
        let track_id = track.id;

        let mut out_buf = VecDeque::with_capacity(2 * frames_per_batch);
        let mut frame_count = 0;

        let mut resample_and_output = |batch: &[f32]| {
            let resampled = resampler.process(&[batch], None).unwrap();

            let resampled = resampled[0]
                .iter()
                .map(|sample| {
                    let s = (sample * (1i32 << 31) as f32) as i32;
                    (s >> 16) as i16
                })
                .collect::<Vec<_>>();
            writer.push(resampled.as_slice()).unwrap();
        };

        let mut progress_bar =
            progressing::mapping::Bar::with_range(0, len_estimated as i64).timed();

        // The decode loop.
        loop {
            // Get the next packet from the media format.
            let packet = match format.next_packet() {
                Ok(packet) => packet,
                Err(Error::IoError(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    // Done
                    break;
                }
                Err(err) => {
                    // A unrecoverable error occured, halt decoding.
                    panic!("{}", err);
                }
            };

            // Consume any new metadata that has been read since the last packet.
            while !format.metadata().is_latest() {
                // Pop the old head of the metadata queue.
                format.metadata().pop();

                // Consume the new metadata at the head of the metadata queue.
            }

            // If the packet does not belong to the selected track, skip over it.
            if packet.track_id() != track_id {
                continue;
            }

            // Decode the packet into audio samples.
            match decoder.decode(&packet) {
                Ok(decoded) => {
                    let num_channels = decoded.spec().channels.count();
                    let mut buf = decoded.make_equivalent::<f32>();

                    decoded.convert(&mut buf);
                    match num_channels {
                        1 => {
                            for v in buf.chan(0) {
                                out_buf.push_back(*v);
                            }
                        }
                        2 => {
                            let (l, r) = buf.chan_pair_mut(0, 1);
                            for (l, r) in l.iter().zip(r.iter()) {
                                let sum = l + r;
                                out_buf.push_back(sum * 0.5);
                            }
                        }
                        o => panic!("Unsupported number of channels: {}", o),
                    }
                    if out_buf.len() > frames_per_batch {
                        let batch = out_buf.drain(..frames_per_batch).collect::<Vec<_>>();
                        resample_and_output(&batch);
                        frame_count += batch.len();
                        progress_bar.set(frame_count as i64);
                        if progress_bar.has_progressed_significantly() {
                            print!("\r{}", progress_bar);
                        }
                    }

                    //out_file.write_all(out_buf).unwrap();
                    //println!("Written {} samples", out_buf.len());
                }
                Err(Error::IoError(_)) => {
                    // The packet failed to decode due to an IO error, skip the packet.
                    continue;
                }
                Err(Error::DecodeError(_)) => {
                    // The packet failed to decode due to invalid data, skip the packet.
                    continue;
                }
                Err(err) => {
                    // An unrecoverable error occured, halt decoding.
                    panic!("{:?}", err);
                }
            }
        }
        let batch = out_buf
            .drain(..)
            .chain(std::iter::repeat(0.0))
            .take(frames_per_batch)
            .collect::<Vec<_>>();
        resample_and_output(&batch);

        writer.flush().unwrap();
    }

    std::fs::rename(destination_tmp, destination).unwrap();

    println!(" Done");
}

fn sync_v1(source: &Path, destination: &Path) {
    let md_file_in = source.join("media_definition.txt");
    let md = media_definition::load_media_definition(&md_file_in, source);
    let md_file_out = destination.join("media_definition.txt");
    let mut md_file_out = std::fs::File::create(md_file_out).unwrap();

    const EXT: &str = "ogg";

    for (id, name) in &md {
        let dst_file = destination
            .join(name.file_name().unwrap())
            .with_extension(EXT);
        let file_name = dst_file.file_name().unwrap().to_str().unwrap().to_owned();

        if dst_file.exists() {
            println!(
                "File {} ({}) exists",
                dst_file.to_string_lossy(),
                name.to_string_lossy()
            );
        } else {
            let src_file = source.join(name);
            if src_file.extension() == Some(std::ffi::OsStr::new(EXT)) {
                println!(
                    "Copying {} ({})",
                    dst_file.to_string_lossy(),
                    name.to_string_lossy()
                );
                std::fs::copy(src_file, dst_file).unwrap();
            } else {
                println!(
                    "Transcoding {} ({})",
                    dst_file.to_string_lossy(),
                    name.to_string_lossy()
                );
                transcode_v1(&src_file, &dst_file);
            }
        }

        use std::io::Write;
        let _ = writeln!(md_file_out, "0x{} {}", id, file_name);
    }
}

fn sync_v2(source: &Path, destination: &Path) {
    let md_file = source.join("media_definition.txt");
    let md = media_definition::load_media_definition(&md_file, source);

    for (id, name) in &md {
        let dst_file = destination.join(&*id.filename_v2());
        if dst_file.exists() {
            println!(
                "File {} ({}) exists",
                dst_file.to_string_lossy(),
                name.to_string_lossy()
            );
        } else {
            println!(
                "Transcoding {} ({})",
                dst_file.to_string_lossy(),
                name.to_string_lossy()
            );
            let src_file = source.join(name);
            transcode_v2(&src_file, &dst_file);
        }
    }
}

fn main() {
    // Get the first command line argument.
    let args = Args::parse();

    let version = args.version_overwrite.unwrap_or_else(|| {
        let mut md = args.destination.clone();
        md.push("media_definition.txt");
        if md.exists() {
            println!("Inferring a media directory v1");
            Version::V1
        } else {
            println!("Inferring a media directory v2");
            Version::V2
        }
    });

    match version {
        Version::V1 => {
            sync_v1(&args.source, &args.destination);
        }
        Version::V2 => {
            sync_v2(&args.source, &args.destination);
        }
    }
}
