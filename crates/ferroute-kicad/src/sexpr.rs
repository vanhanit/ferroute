// SPDX-License-Identifier: AGPL-3.0-only
//! Lossless S-expression tree plus a KiCad-style pretty printer.
//!
//! The grammar is KiCad's (`DSNLEXER` in non-Specctra mode): lists in parentheses, bare
//! atoms terminated by whitespace or a parenthesis, and double-quoted strings with the
//! escapes `\" \\ \a \b \f \n \r \t \v \xHH \ooo`. KiCad board files have no comments.
//!
//! Atom text is stored *decoded* (escapes resolved) together with a `quoted` flag, so a
//! number written bare (`1.0`) can be told apart from the string `"1.0"` (this matters for
//! the net dialect: `(net 1 "GND")` vs `(net "1")`).
//!
//! The printer mimics `KICAD_FORMAT::Prettify` (`common/io/kicad/kicad_io_utils.cpp`):
//! tab indentation, a list without sub-lists stays on one line, a list with sub-lists puts
//! each sub-list on its own line and closes on its own line, consecutive `(xy ..)` lists
//! share a line until column 99, and a flat list wraps after column 72.

use crate::KicadError;

/// Maximum nesting depth accepted by [`parse`].
const MAX_DEPTH: usize = 512;
/// `consecutiveTokenWrapThreshold` of `Prettify`.
const WRAP_COLUMN: usize = 72;
/// `xySpecialCaseColumnLimit` of `Prettify`.
const XY_COLUMN: usize = 99;

/// One S-expression node.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SExpr {
    List(Vec<SExpr>),
    Atom(Atom),
}

/// A token: decoded text plus whether it was written as a quoted string.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Atom {
    pub text: String,
    pub quoted: bool,
}

impl SExpr {
    /// A bare (unquoted) atom, e.g. a number or keyword.
    #[must_use]
    pub fn sym(text: impl Into<String>) -> Self {
        Self::Atom(Atom { text: text.into(), quoted: false })
    }

    /// A quoted string atom.
    #[must_use]
    pub fn quoted(text: impl Into<String>) -> Self {
        Self::Atom(Atom { text: text.into(), quoted: true })
    }

    /// A list node.
    #[must_use]
    pub const fn list(items: Vec<Self>) -> Self {
        Self::List(items)
    }

    /// The elements of a list node (`None` for atoms).
    #[must_use]
    pub fn items(&self) -> Option<&[Self]> {
        match self {
            Self::List(v) => Some(v),
            Self::Atom(_) => None,
        }
    }

    /// Mutable elements of a list node.
    pub fn items_mut(&mut self) -> Option<&mut Vec<Self>> {
        match self {
            Self::List(v) => Some(v),
            Self::Atom(_) => None,
        }
    }

    /// Decoded text of an atom node.
    #[must_use]
    pub fn text(&self) -> Option<&str> {
        match self {
            Self::Atom(a) => Some(&a.text),
            Self::List(_) => None,
        }
    }

    /// `true` for an unquoted atom with exactly this text.
    #[must_use]
    pub fn is_sym(&self, s: &str) -> bool {
        matches!(self, Self::Atom(a) if !a.quoted && a.text == s)
    }

    /// The first element of a list when it is an atom: `(at 1 2)` has head `at`.
    #[must_use]
    pub fn head(&self) -> Option<&str> {
        self.items()?.first()?.text()
    }

    /// First direct sub-list whose head is `name`.
    #[must_use]
    pub fn child(&self, name: &str) -> Option<&Self> {
        self.items()?.iter().find(|c| c.head() == Some(name) && c.items().is_some())
    }

    /// Mutable variant of [`SExpr::child`].
    pub fn child_mut(&mut self, name: &str) -> Option<&mut Self> {
        self.items_mut()?.iter_mut().find(|c| c.items().is_some() && c.head() == Some(name))
    }

    /// All direct sub-lists whose head is `name`.
    pub fn children<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Self> + 'a {
        self.items()
            .unwrap_or(&[])
            .iter()
            .filter(move |c| c.items().is_some() && c.head() == Some(name))
    }

    /// Text of the `i`-th element of this list if it is an atom. Index 0 is the head:
    /// for `(at 1 2)`, `atom(1) == Some("1")`.
    #[must_use]
    pub fn atom(&self, i: usize) -> Option<&str> {
        self.items()?.get(i)?.text()
    }

    /// Serializes in KiCad 10 style (see the module documentation). Lists end with a
    /// newline at top level. Unquoted atoms that cannot be written bare (empty, containing
    /// whitespace, parentheses or starting with `"`) are written quoted instead.
    #[must_use]
    pub fn serialize(&self) -> String {
        let mut w = Writer { out: String::new(), line_start: 0 };
        w.node(self, 0);
        w.out.push('\n');
        w.out
    }
}

fn needs_quotes(a: &Atom) -> bool {
    a.quoted
        || a.text.is_empty()
        || a.text.starts_with('"')
        || a.text.chars().any(|c| c.is_whitespace() || c == '(' || c == ')')
}

fn write_atom(out: &mut String, a: &Atom) {
    if !needs_quotes(a) {
        out.push_str(&a.text);
        return;
    }
    out.push('"');
    for c in a.text.chars() {
        match c {
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            _ => out.push(c),
        }
    }
    out.push('"');
}

struct Writer {
    out: String,
    /// Byte offset of the start of the current output line.
    line_start: usize,
}

impl Writer {
    fn col(&self) -> usize {
        self.out.len() - self.line_start
    }

    fn newline(&mut self, depth: usize) {
        self.out.push('\n');
        self.line_start = self.out.len();
        for _ in 0..depth {
            self.out.push('\t');
        }
    }

    fn node(&mut self, n: &SExpr, depth: usize) {
        match n {
            SExpr::Atom(a) => write_atom(&mut self.out, a),
            SExpr::List(items) => self.list(items, depth),
        }
    }

    fn list(&mut self, items: &[SExpr], depth: usize) {
        self.out.push('(');
        let is_xy = items.first().is_some_and(|h| h.is_sym("xy"));
        let mut wrapped = false;
        let mut prev_xy_list = false;
        let mut last_is_list = false;
        for (i, item) in items.iter().enumerate() {
            match item {
                SExpr::Atom(a) => {
                    if i > 0 {
                        if !is_xy && self.col() >= WRAP_COLUMN {
                            self.newline(depth + 1);
                            wrapped = true;
                        } else {
                            self.out.push(' ');
                        }
                    }
                    write_atom(&mut self.out, a);
                    prev_xy_list = false;
                    last_is_list = false;
                }
                SExpr::List(sub) => {
                    let sub_xy = sub.first().is_some_and(|h| h.is_sym("xy"));
                    if sub_xy && prev_xy_list && self.col() < XY_COLUMN {
                        self.out.push(' ');
                    } else {
                        self.newline(depth + 1);
                    }
                    self.list(sub, depth + 1);
                    prev_xy_list = sub_xy;
                    last_is_list = true;
                }
            }
        }
        if last_is_list || wrapped {
            self.newline(depth);
        }
        self.out.push(')');
    }
}

struct Cursor<'a> {
    b: &'a [u8],
    i: usize,
    line: usize,
    col: usize,
}

impl Cursor<'_> {
    fn err(&self, msg: impl Into<String>) -> KicadError {
        KicadError::Parse { line: self.line, col: self.col, msg: msg.into() }
    }

    fn bump(&mut self) -> Option<u8> {
        let c = *self.b.get(self.i)?;
        self.i += 1;
        if c == b'\n' {
            self.line += 1;
            self.col = 1;
        } else if c & 0xC0 != 0x80 {
            self.col += 1;
        }
        Some(c)
    }

    fn peek(&self) -> Option<u8> {
        self.b.get(self.i).copied()
    }
}

fn is_ws(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\r' | b'\n' | 0x0b | 0x0c)
}

fn lex_string(cur: &mut Cursor<'_>) -> Result<String, KicadError> {
    // The opening quote has been consumed.
    let mut bytes: Vec<u8> = Vec::new();
    loop {
        let Some(c) = cur.bump() else {
            return Err(cur.err("unterminated string"));
        };
        match c {
            b'"' => break,
            b'\\' => {
                let Some(e) = cur.bump() else {
                    return Err(cur.err("unterminated string"));
                };
                match e {
                    b'"' | b'\\' => bytes.push(e),
                    b'a' => bytes.push(0x07),
                    b'b' => bytes.push(0x08),
                    b'f' => bytes.push(0x0c),
                    b'n' => bytes.push(b'\n'),
                    b'r' => bytes.push(b'\r'),
                    b't' => bytes.push(b'\t'),
                    b'v' => bytes.push(0x0b),
                    b'x' => {
                        let mut v = 0u32;
                        let mut n = 0;
                        while n < 2 {
                            match cur.peek().and_then(|d| char::from(d).to_digit(16)) {
                                Some(d) => {
                                    v = v * 16 + d;
                                    cur.bump();
                                    n += 1;
                                }
                                None => break,
                            }
                        }
                        if n == 0 {
                            bytes.push(b'x');
                        } else {
                            bytes.push(u8::try_from(v).unwrap_or(b'?'));
                        }
                    }
                    b'0'..=b'7' => {
                        let mut v = u32::from(e - b'0');
                        let mut n = 1;
                        while n < 3 {
                            match cur.peek() {
                                Some(d @ b'0'..=b'7') => {
                                    v = v * 8 + u32::from(d - b'0');
                                    cur.bump();
                                    n += 1;
                                }
                                _ => break,
                            }
                        }
                        bytes.push(u8::try_from(v & 0xff).unwrap_or(b'?'));
                    }
                    other => {
                        // KiCad: a goofed escape is interpreted as a backslash.
                        bytes.push(b'\\');
                        bytes.push(other);
                    }
                }
            }
            _ => bytes.push(c),
        }
    }
    Ok(match String::from_utf8(bytes) {
        Ok(s) => s,
        Err(e) => String::from_utf8_lossy(e.as_bytes()).into_owned(),
    })
}

/// Parses one top-level S-expression list (the whole file).
///
/// # Errors
/// [`KicadError::Parse`] with the line/column of the problem.
pub fn parse(src: &str) -> Result<SExpr, KicadError> {
    let mut cur = Cursor { b: src.as_bytes(), i: 0, line: 1, col: 1 };
    let mut stack: Vec<Vec<SExpr>> = Vec::new();
    let mut root: Option<SExpr> = None;
    while let Some(c) = cur.peek() {
        if is_ws(c) {
            cur.bump();
            continue;
        }
        if root.is_some() {
            return Err(cur.err("unexpected data after the top-level list"));
        }
        match c {
            b'(' => {
                cur.bump();
                if stack.len() >= MAX_DEPTH {
                    return Err(cur.err("nesting too deep"));
                }
                stack.push(Vec::new());
            }
            b')' => {
                let Some(done) = stack.pop() else {
                    return Err(cur.err("unbalanced ')'"));
                };
                cur.bump();
                let node = SExpr::List(done);
                match stack.last_mut() {
                    Some(parent) => parent.push(node),
                    None => root = Some(node),
                }
            }
            _ => {
                let atom = if c == b'"' {
                    cur.bump();
                    SExpr::quoted(lex_string(&mut cur)?)
                } else {
                    let start = cur.i;
                    while let Some(d) = cur.peek() {
                        if is_ws(d) || d == b'(' || d == b')' {
                            break;
                        }
                        cur.bump();
                    }
                    SExpr::sym(String::from_utf8_lossy(&cur.b[start..cur.i]).into_owned())
                };
                match stack.last_mut() {
                    Some(parent) => parent.push(atom),
                    None => return Err(cur.err("expected '(' at top level")),
                }
            }
        }
    }
    if !stack.is_empty() {
        return Err(cur.err("unexpected end of input: unclosed '('"));
    }
    root.ok_or_else(|| cur.err("empty input"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn parses_nested_and_helpers() {
        let t = parse("(a (b 1 2) (b 3 \"x y\") c)").unwrap();
        assert_eq!(t.head(), Some("a"));
        assert_eq!(t.children("b").count(), 2);
        assert_eq!(t.child("b").unwrap().atom(2), Some("2"));
        assert_eq!(t.children("b").nth(1).unwrap().atom(2), Some("x y"));
        assert!(t.child("b").unwrap().items().unwrap()[1].text() == Some("1"));
        assert!(t.items().unwrap()[3].is_sym("c"));
        assert!(t.child("zzz").is_none());
    }

    #[test]
    fn decodes_escapes_like_kicad() {
        let t = parse(r#"(s "a\"b\\c\nd\re\tf\x41\101\q")"#).unwrap();
        assert_eq!(t.atom(1), Some("a\"b\\c\nd\re\tfAA\\q"));
        assert!(matches!(&t.items().unwrap()[1], SExpr::Atom(a) if a.quoted));
    }

    #[test]
    fn quoted_and_bare_differ() {
        let t = parse("(net 1 \"1\")").unwrap();
        let it = t.items().unwrap();
        assert!(matches!(&it[1], SExpr::Atom(a) if !a.quoted));
        assert!(matches!(&it[2], SExpr::Atom(a) if a.quoted));
    }

    #[test]
    fn reports_error_positions() {
        assert!(matches!(parse("(a\n  (b"), Err(KicadError::Parse { line: 2, .. })));
        assert!(matches!(parse("(a))"), Err(KicadError::Parse { .. })));
        assert!(matches!(parse("(a \"x"), Err(KicadError::Parse { .. })));
        assert!(matches!(parse("a"), Err(KicadError::Parse { .. })));
        assert!(matches!(parse(""), Err(KicadError::Parse { .. })));
        assert!(matches!(parse("(a) (b)"), Err(KicadError::Parse { .. })));
        let deep = "(".repeat(MAX_DEPTH + 2);
        assert!(matches!(parse(&deep), Err(KicadError::Parse { .. })));
    }

    #[test]
    fn pretty_printer_matches_kicad_layout() {
        let src =
            "(kicad_pcb (version 20260206) (pts (xy 1 2) (xy 3 4)) (foo (bar 1) tail) (empty))";
        let t = parse(src).unwrap();
        let expect = "(kicad_pcb\n\t(version 20260206)\n\t(pts\n\t\t(xy 1 2) (xy 3 4)\n\t)\n\t(foo\n\t\t(bar 1) tail)\n\t(empty)\n)\n";
        assert_eq!(t.serialize(), expect);
        assert_eq!(parse(&t.serialize()).unwrap(), t);
    }

    #[test]
    fn escapes_on_output() {
        let t = SExpr::list(vec![SExpr::sym("s"), SExpr::quoted("a\"b\\c\nd"), SExpr::sym("")]);
        assert_eq!(t.serialize(), "(s \"a\\\"b\\\\c\\nd\" \"\")\n");
    }

    fn arb_atom() -> impl Strategy<Value = SExpr> {
        prop_oneof![
            "[A-Za-z0-9_.+*&-][A-Za-z0-9_.+*&#-]{0,8}".prop_map(SExpr::sym),
            ".{0,12}".prop_map(SExpr::quoted),
            proptest::string::string_regex("[ \"\\\\\n\r\t()a-z]{0,10}")
                .unwrap()
                .prop_map(SExpr::quoted),
        ]
    }

    fn arb_tree() -> impl Strategy<Value = SExpr> {
        let leaf = arb_atom();
        let node = leaf.prop_recursive(5, 80, 6, |inner| {
            proptest::collection::vec(inner, 0..7).prop_map(SExpr::List)
        });
        // The root of a file is always a list.
        proptest::collection::vec(node, 0..6).prop_map(SExpr::List)
    }

    proptest! {
        #[test]
        fn roundtrip(t in arb_tree()) {
            let s = t.serialize();
            prop_assert_eq!(parse(&s).unwrap(), t);
        }

        #[test]
        fn serialize_is_idempotent(t in arb_tree()) {
            let s = t.serialize();
            prop_assert_eq!(parse(&s).unwrap().serialize(), s);
        }
    }
}
