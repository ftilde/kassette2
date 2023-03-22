use id_reader::Id;
use std::collections::HashMap;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

fn parse_num(s: &str) -> Option<u32> {
    match s.as_bytes() {
        [b'0', b'b', ..] => u32::from_str_radix(&s[2..], 2),
        [b'0', b'x', ..] => u32::from_str_radix(&s[2..], 16),
        [b'0', b'o', ..] => u32::from_str_radix(&s[2..], 8),
        _ => u32::from_str_radix(s, 10),
    }
    .ok()
}
fn parse_line(l: &str) -> Option<(Id, PathBuf)> {
    let end = l.find(" ")?;
    let uid_str = &l[..end];
    let path_str = l[end..].trim();
    let id = Id(parse_num(uid_str)?.to_be_bytes());
    let path = PathBuf::from(path_str);
    Some((id, path))
}

pub fn load_media_definition(
    map_definition_file: impl AsRef<Path>,
    media_file_root: impl AsRef<Path>,
) -> HashMap<Id, PathBuf> {
    let f = std::fs::File::open(map_definition_file).unwrap(); // If this fails we cannot do anything anyways.
    parse_media_definition(f, media_file_root)
}

pub fn parse_media_definition(
    src: impl std::io::Read,
    media_file_root: impl AsRef<Path>,
) -> HashMap<Id, PathBuf> {
    let f = BufReader::new(src);
    let media_file_root = media_file_root.as_ref();

    let mut map = HashMap::new();

    for l in f.lines() {
        let l = match l {
            Ok(l) => l,
            Err(_) => continue,
        };
        let l = l.trim_start();
        if l.is_empty() || l.starts_with("#") {
            continue;
        }
        if let Some((uid, path)) = parse_line(l) {
            map.insert(uid, media_file_root.join(path));
        }
    }
    map
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_parse_num() {
        assert_eq!(parse_num("aa"), None);
        assert_eq!(parse_num("0xdeadbeef"), Some(0xdeadbeef));
        assert_eq!(parse_num("0b11"), Some(0b11));
        assert_eq!(parse_num("0o11"), Some(0o11));
        assert_eq!(parse_num("37"), Some(37));
    }

    #[test]
    fn test_parse_line() {
        assert_eq!(parse_line(""), None);
        assert_eq!(parse_line(" "), None);
        assert_eq!(parse_line("bla"), None);
        assert_eq!(parse_line("123"), None);
        assert_eq!(
            parse_line("123 /foo/bar"),
            Some((Uid(123), PathBuf::from("/foo/bar")))
        );
        assert_eq!(
            parse_line("0x42 baz"),
            Some((Uid(0x42), PathBuf::from("baz")))
        );
    }

    #[test]
    fn test_parse_media_definition() {
        let f = std::io::Cursor::new(
            &r"
            0x123 foo/bar
            456 /bla/

            #013 commented out

            # just some comment
            0xcafe cafe.ogg
            "[..],
        );
        let m = parse_media_definition(f, "/root/");
        assert_eq!(m.len(), 3);
        assert_eq!(m.get(&Uid(0x123)).unwrap(), &PathBuf::from("/root/foo/bar"));
        assert_eq!(m.get(&Uid(456)).unwrap(), &PathBuf::from("/bla"));
        assert_eq!(
            m.get(&Uid(0xcafe)).unwrap(),
            &PathBuf::from("/root/cafe.ogg")
        );
    }
}
