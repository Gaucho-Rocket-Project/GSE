//! Tiny JSON reader/writer.
//!
//! The ground app deliberately has **no external crates**: it must build and
//! run offline on Windows, Linux and macOS. Request bodies are small (settings,
//! keyboard bindings), so a compact hand-written parser is enough.

use std::fmt::Write as _;

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Number(f64),
    String(String),
    Array(Vec<Value>),
    Object(Vec<(String, Value)>),
}

impl Value {
    pub fn get(&self, key: &str) -> Option<&Value> {
        match self {
            Value::Object(items) => items.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::String(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Number(n) => Some(*n),
            _ => None,
        }
    }

    pub fn as_i64(&self) -> Option<i64> {
        self.as_f64().map(|n| n as i64)
    }

    pub fn as_u64(&self) -> Option<u64> {
        self.as_f64().and_then(|n| {
            if n >= 0.0 && n <= u64::MAX as f64 {
                Some(n as u64)
            } else {
                None
            }
        })
    }

    /// Object entries, if this is an object.
    pub fn entries(&self) -> &[(String, Value)] {
        match self {
            Value::Object(items) => items,
            _ => &[],
        }
    }
}

pub fn parse(input: &str) -> Result<Value, String> {
    let mut parser = Parser {
        bytes: input.as_bytes(),
        pos: 0,
    };
    let value = parser.value()?;
    parser.skip_ws();
    if parser.pos != parser.bytes.len() {
        return Err(format!("trailing characters at byte {}", parser.pos));
    }
    Ok(value)
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn bump(&mut self) -> Option<u8> {
        let b = self.peek()?;
        self.pos += 1;
        Some(b)
    }

    fn skip_ws(&mut self) {
        while let Some(b) = self.peek() {
            if matches!(b, b' ' | b'\t' | b'\r' | b'\n') {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    fn value(&mut self) -> Result<Value, String> {
        self.skip_ws();
        match self.peek() {
            Some(b'{') => self.object(),
            Some(b'[') => self.array(),
            Some(b'"') => Ok(Value::String(self.string()?)),
            Some(b't') => {
                self.literal(b"true")?;
                Ok(Value::Bool(true))
            }
            Some(b'f') => {
                self.literal(b"false")?;
                Ok(Value::Bool(false))
            }
            Some(b'n') => {
                self.literal(b"null")?;
                Ok(Value::Null)
            }
            Some(b'-') | Some(b'0'..=b'9') => self.number(),
            Some(other) => Err(format!(
                "unexpected character '{}' at byte {}",
                other as char, self.pos
            )),
            None => Err("unexpected end of input".to_string()),
        }
    }

    fn literal(&mut self, text: &[u8]) -> Result<(), String> {
        if self.bytes.len() >= self.pos + text.len()
            && &self.bytes[self.pos..self.pos + text.len()] == text
        {
            self.pos += text.len();
            Ok(())
        } else {
            Err(format!("invalid literal at byte {}", self.pos))
        }
    }

    fn object(&mut self) -> Result<Value, String> {
        self.bump(); // {
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.bump();
            return Ok(Value::Object(items));
        }
        loop {
            self.skip_ws();
            let key = match self.peek() {
                Some(b'"') => self.string()?,
                _ => return Err(format!("expected object key at byte {}", self.pos)),
            };
            self.skip_ws();
            if self.bump() != Some(b':') {
                return Err(format!("expected ':' at byte {}", self.pos));
            }
            let value = self.value()?;
            items.push((key, value));
            self.skip_ws();
            match self.bump() {
                Some(b',') => continue,
                Some(b'}') => break,
                _ => return Err(format!("expected ',' or '}}' at byte {}", self.pos)),
            }
        }
        Ok(Value::Object(items))
    }

    fn array(&mut self) -> Result<Value, String> {
        self.bump(); // [
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.bump();
            return Ok(Value::Array(items));
        }
        loop {
            items.push(self.value()?);
            self.skip_ws();
            match self.bump() {
                Some(b',') => continue,
                Some(b']') => break,
                _ => return Err(format!("expected ',' or ']' at byte {}", self.pos)),
            }
        }
        Ok(Value::Array(items))
    }

    fn string(&mut self) -> Result<String, String> {
        if self.bump() != Some(b'"') {
            return Err(format!("expected '\"' at byte {}", self.pos));
        }
        let mut out = String::new();
        loop {
            match self.bump() {
                Some(b'"') => return Ok(out),
                Some(b'\\') => match self.bump() {
                    Some(b'"') => out.push('"'),
                    Some(b'\\') => out.push('\\'),
                    Some(b'/') => out.push('/'),
                    Some(b'b') => out.push('\u{0008}'),
                    Some(b'f') => out.push('\u{000C}'),
                    Some(b'n') => out.push('\n'),
                    Some(b'r') => out.push('\r'),
                    Some(b't') => out.push('\t'),
                    Some(b'u') => {
                        let hi = self.hex4()?;
                        let ch = if (0xD800..=0xDBFF).contains(&hi) {
                            // Surrogate pair.
                            if self.bump() != Some(b'\\') || self.bump() != Some(b'u') {
                                return Err("invalid surrogate pair".to_string());
                            }
                            let lo = self.hex4()?;
                            if !(0xDC00..=0xDFFF).contains(&lo) {
                                return Err("invalid low surrogate".to_string());
                            }
                            let code = 0x10000 + ((hi - 0xD800) << 10) + (lo - 0xDC00);
                            char::from_u32(code).unwrap_or('\u{FFFD}')
                        } else {
                            char::from_u32(hi).unwrap_or('\u{FFFD}')
                        };
                        out.push(ch);
                    }
                    Some(other) => {
                        return Err(format!("invalid escape '\\{}'", other as char));
                    }
                    None => return Err("truncated escape".to_string()),
                },
                Some(b) if b < 0x20 => return Err("control character in string".to_string()),
                Some(b) => {
                    // Re-decode UTF-8 sequences one byte at a time.
                    if b < 0x80 {
                        out.push(b as char);
                    } else {
                        let start = self.pos - 1;
                        let width = utf8_width(b);
                        if start + width > self.bytes.len() {
                            return Err("truncated UTF-8 sequence".to_string());
                        }
                        match std::str::from_utf8(&self.bytes[start..start + width]) {
                            Ok(s) => {
                                out.push_str(s);
                                self.pos = start + width;
                            }
                            Err(_) => return Err("invalid UTF-8".to_string()),
                        }
                    }
                }
                None => return Err("unterminated string".to_string()),
            }
        }
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let mut value = 0u32;
        for _ in 0..4 {
            let b = self.bump().ok_or("truncated \\u escape")?;
            let digit = match b {
                b'0'..=b'9' => (b - b'0') as u32,
                b'a'..=b'f' => (b - b'a' + 10) as u32,
                b'A'..=b'F' => (b - b'A' + 10) as u32,
                _ => return Err("invalid hex digit in \\u escape".to_string()),
            };
            value = value * 16 + digit;
        }
        Ok(value)
    }

    fn number(&mut self) -> Result<Value, String> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.bump();
        }
        while let Some(b) = self.peek() {
            if b.is_ascii_digit() || matches!(b, b'.' | b'e' | b'E' | b'+' | b'-') {
                self.bump();
            } else {
                break;
            }
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos])
            .map_err(|_| "invalid number".to_string())?;
        text.parse::<f64>()
            .map(Value::Number)
            .map_err(|_| format!("invalid number '{text}'"))
    }
}

fn utf8_width(first: u8) -> usize {
    match first {
        0x00..=0x7F => 1,
        0xC0..=0xDF => 2,
        0xE0..=0xEF => 3,
        _ => 4,
    }
}

/// Append a JSON-quoted string.
pub fn escape_into(out: &mut String, value: &str) {
    out.push('"');
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0C}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

pub fn quoted(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    escape_into(&mut out, value);
    out
}

/// Append `key: value` to a JSON object under construction.
pub fn field(out: &mut String, key: &str, value_json: &str) {
    if !out.ends_with('{') && !out.ends_with(',') {
        out.push(',');
    }
    escape_into(out, key);
    out.push(':');
    out.push_str(value_json);
}

pub fn bool_json(value: bool) -> &'static str {
    if value {
        "true"
    } else {
        "false"
    }
}

pub fn opt_str_json(value: Option<&str>) -> String {
    match value {
        Some(s) => quoted(s),
        None => "null".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_objects() {
        let value = parse(r#"{"a":1,"b":[true,null,"x"],"c":{"d":-2.5e2}}"#).unwrap();
        assert_eq!(value.get("a").unwrap().as_u64(), Some(1));
        assert_eq!(value.get("b").unwrap(), &Value::Array(vec![
            Value::Bool(true),
            Value::Null,
            Value::String("x".into())
        ]));
        assert_eq!(value.get("c").unwrap().get("d").unwrap().as_f64(), Some(-250.0));
    }

    #[test]
    fn parses_escapes() {
        let value = parse(r#"{"s":"a\"b\\c\nd\u00e9\ud83d\ude80"}"#).unwrap();
        assert_eq!(
            value.get("s").unwrap().as_str().unwrap(),
            "a\"b\\c\ndé🚀"
        );
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse("{").is_err());
        assert!(parse("{} trailing").is_err());
        assert!(parse(r#"{"a":}"#).is_err());
        assert!(parse(r#"{"a" 1}"#).is_err());
    }

    #[test]
    fn escapes_output() {
        let mut out = String::new();
        escape_into(&mut out, "quote\" backslash\\ newline\n tab\t bell\u{7}");
        assert_eq!(out, "\"quote\\\" backslash\\\\ newline\\n tab\\t bell\\u0007\"");
        assert_eq!(quoted("hello"), "\"hello\"");
        assert_eq!(opt_str_json(None), "null");
        assert_eq!(opt_str_json(Some("x")), "\"x\"");
    }

    #[test]
    fn builds_fields() {
        let mut out = String::from("{");
        field(&mut out, "a", "1");
        field(&mut out, "b", "true");
        out.push('}');
        assert_eq!(parse(&out).unwrap().get("b").unwrap().as_bool(), Some(true));
    }

    #[test]
    fn numbers_and_bools() {
        assert_eq!(parse("42").unwrap(), Value::Number(42.0));
        assert_eq!(parse("true").unwrap(), Value::Bool(true));
        assert_eq!(parse("null").unwrap(), Value::Null);
        assert!(parse("01").unwrap().as_f64().is_some());
    }
}
