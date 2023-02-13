use clap::Parser;
use flac_bound::{FlacEncoder, WriteWrapper};
use std::{fs::File, io::Write, path::PathBuf};
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
#[command()] // Just to make testing across clap features easier
struct Args {
    #[arg()]
    file: PathBuf,
}

fn main() {
    // Get the first command line argument.
    let args = Args::parse();

    // Open the media source.
    let src = File::open(&args.file).expect("failed to open media");

    let out_file = File::create(args.file.with_extension("flc").file_name().unwrap()).unwrap();
    let mut out_file = std::io::BufWriter::new(out_file);

    {
        let mut outw = WriteWrapper(&mut out_file);
        let mut enc = FlacEncoder::new()
            .unwrap()
            .compression_level(8)
            .channels(1)
            .bits_per_sample(8)
            .sample_rate(40000)
            .verify(true)
            .init_write(&mut outw)
            .unwrap();

        // Create the media source stream.
        let mss = MediaSourceStream::new(Box::new(src), Default::default());

        // Create a probe hint using the file's extension. [Optional]
        let mut hint = Hint::new();
        if let Some(ext) = args.file.extension() {
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

        // Use the default options for the decoder.
        let dec_opts: DecoderOptions = Default::default();

        // Create a decoder for the track.
        let mut decoder = symphonia::default::get_codecs()
            .make(&track.codec_params, &dec_opts)
            .expect("unsupported codec");

        // Store the track identifier, it will be used to filter packets.
        let track_id = track.id;

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
                    let mut buf = decoded.make_equivalent::<i8>();
                    decoded.convert(&mut buf);
                    let mut out_buf = Vec::with_capacity(buf.frames());
                    match num_channels {
                        1 => {
                            for v in buf.chan(0) {
                                out_buf.push(*v as i32);
                            }
                        }
                        2 => {
                            let (l, r) = buf.chan_pair_mut(0, 1);
                            for (l, r) in l.iter().zip(r.iter()) {
                                let sum = *l as i32 + *r as i32;
                                out_buf.push(sum >> 1);
                            }
                        }
                        o => panic!("Unsupported number of channels: {}", o),
                    }
                    enc.process(&[&out_buf[..]]).unwrap();

                    //out_file.write_all(out_buf).unwrap();
                    println!("Written {} samples", out_buf.len());
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
        enc.finish().unwrap();
    }

    out_file.flush().unwrap();
}
