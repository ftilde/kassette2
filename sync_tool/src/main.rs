mod media_definition;

use clap::Parser;
use flac_bound::{FlacEncoder, WriteWrapper};
use progressing::Baring;
use rubato::{InterpolationParameters, InterpolationType, Resampler, SincFixedIn, WindowFunction};
use std::{
    collections::VecDeque,
    fs::File,
    io::Write,
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

#[derive(Parser, Debug)] // requires `derive` feature
#[command()]
struct Args {
    #[arg()]
    source: PathBuf,

    #[arg()]
    destination: PathBuf,
}

fn transcode_v2(source: &Path, destination: &Path) {
    // Open the media source.
    let src = File::open(source).expect("failed to open media");

    let out_file = File::create(destination).unwrap();
    let mut out_file = std::io::BufWriter::new(out_file);

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

        let mut outw = WriteWrapper(&mut out_file);
        let mut enc = FlacEncoder::new()
            .unwrap()
            .compression_level(8)
            .channels(1)
            .bits_per_sample(format_bits)
            .streamable_subset(false)
            .sample_rate(target_sample_rate)
            .verify(true)
            .init_write(&mut outw)
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

            let right_shift_amount = 32 - bits;
            let left_shift_amount = format_bits - bits;
            let resampled = resampled[0]
                .iter()
                .map(|sample| {
                    let s = (sample * (1i32 << 31) as f32) as i32;
                    (s >> right_shift_amount) << left_shift_amount
                })
                .collect::<Vec<_>>();
            enc.process(&[resampled.as_slice()]).unwrap();
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

        enc.finish().unwrap();
    }

    out_file.flush().unwrap();
    println!(" Done");
}

fn sync_v2(source: &Path, destination: &Path) {
    let md_file = source.join("media_definition.txt");
    let md = media_definition::load_media_definition(&md_file, source);

    for (id, name) in &md {
        let dst_file = destination.join(id.filename_v2());
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

    sync_v2(&args.source, &args.destination);
}
