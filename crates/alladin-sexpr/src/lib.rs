//! Alladin S-expression parser.
//!
//! `.kicad_pcb` (and every other KiCad save-file format: `.kicad_sch`,
//! `.kicad_mod`, `.kicad_pro`, ...) is one big S-expression: parenthesised
//! lists of symbols, numbers, and quoted strings, no schema baked into
//! the syntax itself -- e.g. `(pad "1" smd rect (at 0 0) (size 0.9 0.95)
//! (net 1 "GND"))`. This crate knows *only* that generic syntax; it has
//! zero knowledge of what a `pad` or a `net` mean. `alladin-kicad-io`
//! builds the KiCad-specific semantic layer (footprints, pads, tracks,
//! nets -> `alladin_core::Node`) on top of the tree this crate produces.
//! Keeping the split this way means the parser is trivially reusable for
//! *any* KiCad sexpr file, not just `.kicad_pcb`, and is fully testable
//! in complete isolation from KiCad's own (much larger, ever-changing)
//! semantics.
//!
//! Deliberately not supported (real `.kicad_pcb` files don't use them,
//! so this isn't a real-world gap): comments, multiple top-level forms
//! in one input (KiCad files are always exactly one big `(...)` form).

use std::fmt;
use thiserror::Error;

/// A parsed S-expression node. Untyped by design -- exactly like KiCad's
/// own writer/reader, which never distinguishes "this token is an
/// integer" from "this token is a float" from "this token is a bare
/// symbol" at the syntax level; that distinction only exists once
/// something semantic (like `alladin-kicad-io`) knows which field it's
/// looking at and calls the matching `as_*` accessor below.
#[derive(Debug, Clone, PartialEq)]
pub enum SExpr {
    /// A bare, unquoted token: `pad`, `smd`, `F.Cu`, `1.5`, `-90`, `at`.
    Sym(String),
    /// A double-quoted token, with `\"` and `\\` escapes resolved:
    /// `"GND"`, `"Resistor_SMD:R_0603_1608Metric"`.
    Str(String),
    /// A parenthesised list of zero or more child nodes.
    List(Vec<SExpr>),
}

#[derive(Debug, Error, PartialEq)]
pub enum ParseError {
    #[error("unexpected end of input while parsing a list (unbalanced '(')")]
    UnclosedList,
    #[error("unexpected end of input inside a quoted string (unbalanced '\"')")]
    UnterminatedString,
    #[error("unexpected ')' with no matching '(' at byte offset {0}")]
    UnmatchedCloseParen(usize),
    #[error("trailing input after the first complete expression, at byte offset {0}")]
    TrailingInput(usize),
    #[error("empty input: nothing to parse")]
    Empty,
}

impl SExpr {
    pub fn as_list(&self) -> Option<&[SExpr]> {
        match self {
            SExpr::List(items) => Some(items),
            _ => None,
        }
    }

    /// The raw text of a `Sym` or `Str` node -- most callers don't care
    /// which kind of atom they got (KiCad itself is inconsistent about
    /// quoting scalars), only its text.
    pub fn text(&self) -> Option<&str> {
        match self {
            SExpr::Sym(s) | SExpr::Str(s) => Some(s),
            SExpr::List(_) => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        self.text()?.parse::<f64>().ok()
    }

    pub fn as_i64(&self) -> Option<i64> {
        self.text()?.parse::<i64>().ok()
    }

    /// If this node is a `List` whose first element is `Sym(head)`,
    /// returns the rest of the list (the "arguments"). This is the
    /// single most common shape in KiCad sexpr files -- `(at 1 2 90)`,
    /// `(size 0.9 0.95)`, `(net 1 "GND")` are all "head + args" lists --
    /// so every semantic extractor ends up calling this constantly.
    pub fn tagged(&self, head: &str) -> Option<&[SExpr]> {
        let items = self.as_list()?;
        match items.first()? {
            SExpr::Sym(s) if s == head => Some(&items[1..]),
            _ => None,
        }
    }

    /// The first direct child list tagged `head`, if this node is a
    /// list. E.g. on a `pad` node, `.child("net")` finds its `(net ...)`
    /// sub-form, if any.
    pub fn child(&self, head: &str) -> Option<&SExpr> {
        self.as_list()?
            .iter()
            .find(|item| item.tagged(head).is_some())
    }

    /// Every direct child list tagged `head`, in order. E.g. on the
    /// board root, `.children("footprint")` iterates every footprint.
    pub fn children<'a>(&'a self, head: &'a str) -> impl Iterator<Item = &'a SExpr> + 'a {
        self.as_list()
            .into_iter()
            .flatten()
            .filter(move |item| item.tagged(head).is_some())
    }
}

impl fmt::Display for SExpr {
    /// Round-trippable-ish pretty printer (single-line, minimal
    /// whitespace) -- not byte-for-byte what KiCad's own writer produces
    /// (no indentation, no field-specific quoting rules), but enough to
    /// eyeball a parsed tree in a test failure or a debug print.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SExpr::Sym(s) => write!(f, "{s}"),
            SExpr::Str(s) => {
                // Mirrors `parse_string`'s decoding exactly (see that
                // function's doc comment): every character re-escaped
                // here is one that can *only* have come from the
                // matching escape sequence during parsing, so this is a
                // true round-trip, not just "looks plausible".
                write!(f, "\"")?;
                for c in s.chars() {
                    match c {
                        '\\' => write!(f, "\\\\")?,
                        '"' => write!(f, "\\\"")?,
                        '\n' => write!(f, "\\n")?,
                        '\t' => write!(f, "\\t")?,
                        '\r' => write!(f, "\\r")?,
                        other => write!(f, "{other}")?,
                    }
                }
                write!(f, "\"")
            }
            SExpr::List(items) => {
                write!(f, "(")?;
                for (i, item) in items.iter().enumerate() {
                    if i > 0 {
                        write!(f, " ")?;
                    }
                    write!(f, "{item}")?;
                }
                write!(f, ")")
            }
        }
    }
}

/// Parse `input` as exactly one top-level S-expression (KiCad files are
/// always a single `(kicad_pcb ...)`-shaped form; trailing non-whitespace
/// content after it is an error, not silently ignored).
pub fn parse(input: &str) -> Result<SExpr, ParseError> {
    let bytes = input.as_bytes();
    let mut pos = 0;
    skip_ws(bytes, &mut pos);
    if pos >= bytes.len() {
        return Err(ParseError::Empty);
    }
    let expr = parse_expr(bytes, &mut pos)?;
    skip_ws(bytes, &mut pos);
    if pos < bytes.len() {
        return Err(ParseError::TrailingInput(pos));
    }
    Ok(expr)
}

fn skip_ws(bytes: &[u8], pos: &mut usize) {
    while *pos < bytes.len() && bytes[*pos].is_ascii_whitespace() {
        *pos += 1;
    }
}

fn is_atom_boundary(b: u8) -> bool {
    b.is_ascii_whitespace() || b == b'(' || b == b')'
}

fn parse_expr(bytes: &[u8], pos: &mut usize) -> Result<SExpr, ParseError> {
    skip_ws(bytes, pos);
    if *pos >= bytes.len() {
        return Err(ParseError::UnclosedList);
    }
    match bytes[*pos] {
        b'(' => parse_list(bytes, pos),
        b')' => Err(ParseError::UnmatchedCloseParen(*pos)),
        b'"' => parse_string(bytes, pos),
        _ => parse_symbol(bytes, pos),
    }
}

fn parse_list(bytes: &[u8], pos: &mut usize) -> Result<SExpr, ParseError> {
    debug_assert_eq!(bytes[*pos], b'(');
    *pos += 1; // consume '('
    let mut items = Vec::new();
    loop {
        skip_ws(bytes, pos);
        if *pos >= bytes.len() {
            return Err(ParseError::UnclosedList);
        }
        if bytes[*pos] == b')' {
            *pos += 1; // consume ')'
            return Ok(SExpr::List(items));
        }
        items.push(parse_expr(bytes, pos)?);
    }
}

fn parse_string(bytes: &[u8], pos: &mut usize) -> Result<SExpr, ParseError> {
    debug_assert_eq!(bytes[*pos], b'"');
    *pos += 1; // consume opening quote
    let mut out = String::new();
    loop {
        if *pos >= bytes.len() {
            return Err(ParseError::UnterminatedString);
        }
        match bytes[*pos] {
            b'"' => {
                *pos += 1;
                return Ok(SExpr::Str(out));
            }
            b'\\' if *pos + 1 < bytes.len() => {
                // Decode to the *actual* character, not the two-byte
                // escape sequence -- this matters for round-tripping.
                // Bug found empirically (see the development log's
                // corresponding update): KiCad's writer represents an
                // embedded newline in multi-line board text (e.g.
                // `(gr_text "Complex hierarchy\nDemo" ...)`) as the
                // literal two-character escape `\n`. The first version
                // of this parser only special-cased `\"` and `\\` and
                // passed any other escape through literally, i.e. it
                // kept the backslash character itself in the decoded
                // string. Combined with `Display` unconditionally
                // escaping every backslash on the way back out, `\n`
                // (one escape, decodes to a single newline char) came
                // back out as `\\n` (an escaped backslash followed by a
                // literal 'n') -- a real KiCad DRC re-run on a
                // round-tripped file confirmed this: the text's
                // rendered width (and therefore its clearance bounding
                // box) changed, producing new clearance violations that
                // had nothing to do with anything Alladin's router
                // itself had touched.
                //
                // Decoding to the real character and having `Display`
                // re-escape based on *that* character (see below) makes
                // the two sides of the round-trip symmetric: any `\\`
                // that ends up in the decoded string only got there via
                // an explicit `\\` escape, so "escape every backslash on
                // the way out" is now actually correct.
                let next = bytes[*pos + 1];
                match next {
                    b'"' => out.push('"'),
                    b'\\' => out.push('\\'),
                    b'n' => out.push('\n'),
                    b't' => out.push('\t'),
                    b'r' => out.push('\r'),
                    // Unrecognised escape (not known to be produced by
                    // KiCad's own writer): drop the backslash and keep
                    // just the character, rather than re-introducing a
                    // backslash that `Display` would then escape again.
                    other => out.push(other as char),
                }
                *pos += 2;
            }
            b => {
                // Safe because we only split on ASCII bytes ('"', '\\')
                // above; any multi-byte UTF-8 sequence in between is
                // copied through untouched byte-by-byte, which is valid
                // since we never split *inside* one.
                out.push(b as char);
                *pos += 1;
            }
        }
    }
}

fn parse_symbol(bytes: &[u8], pos: &mut usize) -> Result<SExpr, ParseError> {
    let start = *pos;
    while *pos < bytes.len() && !is_atom_boundary(bytes[*pos]) {
        *pos += 1;
    }
    // start..*pos is guaranteed to be a valid UTF-8 boundary slice: the
    // only bytes treated specially above are single-byte ASCII
    // whitespace/parens, so any multi-byte UTF-8 character is copied
    // through as part of the symbol, never split.
    let text = std::str::from_utf8(&bytes[start..*pos])
        .expect("atom boundaries are ASCII-only; slice cannot split a UTF-8 sequence")
        .to_string();
    Ok(SExpr::Sym(text))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_flat_list_of_symbols() {
        let parsed = parse("(a b c)").unwrap();
        assert_eq!(
            parsed,
            SExpr::List(vec![
                SExpr::Sym("a".into()),
                SExpr::Sym("b".into()),
                SExpr::Sym("c".into()),
            ])
        );
    }

    #[test]
    fn parses_nested_lists_and_whitespace_insensitively() {
        let a = parse("(at (x 1) (y 2))").unwrap();
        let b = parse("  (  at   ( x 1 ) ( y 2 )  )  ").unwrap();
        assert_eq!(a, b);
        assert_eq!(
            a,
            SExpr::List(vec![
                SExpr::Sym("at".into()),
                SExpr::List(vec![SExpr::Sym("x".into()), SExpr::Sym("1".into())]),
                SExpr::List(vec![SExpr::Sym("y".into()), SExpr::Sym("2".into())]),
            ])
        );
    }

    #[test]
    fn parses_quoted_strings_with_spaces_and_escapes() {
        let parsed = parse(r#"(net 1 "GND net" "with \"quotes\" and a \\backslash")"#).unwrap();
        let items = parsed.as_list().unwrap();
        assert_eq!(items[2], SExpr::Str("GND net".into()));
        assert_eq!(
            items[3],
            SExpr::Str(r#"with "quotes" and a \backslash"#.into())
        );
    }

    #[test]
    fn escaped_newline_round_trips_without_double_escaping() {
        // Regression test for a real bug found via empirical validation
        // against actual KiCad output (see `parse_string`'s and
        // `Display`'s doc comments for the full story): a `\n` escape
        // inside a quoted string must decode to one real newline
        // character, and printing it back out must reproduce the exact
        // original two-character `\n`, not a corrupted `\\n`.
        let original = r#"(gr_text "Complex hierarchy\nDemo" (at 0 0 0))"#;
        let parsed = parse(original).unwrap();
        let text = parsed
            .tagged("gr_text")
            .unwrap()
            .first()
            .unwrap()
            .text()
            .unwrap();
        assert_eq!(text, "Complex hierarchy\nDemo"); // one real newline char
        assert_eq!(
            parsed.to_string(),
            original,
            "must re-escape back to the exact original `\\n`, not `\\\\n`"
        );
    }

    #[test]
    fn parses_numbers_as_symbols_readable_via_as_f64_and_as_i64() {
        let parsed = parse("(at -1.5 2.54 90)").unwrap();
        let items = parsed.as_list().unwrap();
        assert_eq!(items[1].as_f64(), Some(-1.5));
        assert_eq!(items[2].as_f64(), Some(2.54));
        assert_eq!(items[3].as_i64(), Some(90));
    }

    #[test]
    fn tagged_extracts_a_head_symbols_arguments() {
        let parsed = parse(r#"(pad "1" smd rect (at 0 0) (net 3 "GND"))"#).unwrap();
        assert_eq!(
            parsed.tagged("pad").unwrap()[0],
            SExpr::Str("1".into())
        );
        assert!(parsed.tagged("footprint").is_none()); // wrong head
    }

    #[test]
    fn child_and_children_find_nested_tagged_forms() {
        let parsed = parse(
            r#"(footprint "R" (pad "1" smd (net 1 "GND")) (pad "2" smd (net 2 "VCC")))"#,
        )
        .unwrap();

        let net_of_first_pad = parsed
            .children("pad")
            .next()
            .unwrap()
            .child("net")
            .unwrap();
        assert_eq!(net_of_first_pad.tagged("net").unwrap()[0].as_i64(), Some(1));

        assert_eq!(parsed.children("pad").count(), 2);
    }

    #[test]
    fn rejects_unbalanced_parens() {
        assert_eq!(parse("(a (b c)"), Err(ParseError::UnclosedList));
        assert_eq!(
            parse("(a b))"),
            Err(ParseError::TrailingInput(5))
        );
        assert_eq!(parse(")"), Err(ParseError::UnmatchedCloseParen(0)));
    }

    #[test]
    fn rejects_unterminated_string() {
        assert_eq!(parse(r#"(net 1 "GND)"#), Err(ParseError::UnterminatedString));
    }

    #[test]
    fn rejects_empty_input() {
        assert_eq!(parse(""), Err(ParseError::Empty));
        assert_eq!(parse("   \n  "), Err(ParseError::Empty));
    }

    #[test]
    fn display_round_trips_structurally() {
        let original = r#"(pad "1" smd (at 0 0) (net 3 "GND"))"#;
        let parsed = parse(original).unwrap();
        let printed = parsed.to_string();
        // Not byte-identical (no guarantee on exact spacing), but must
        // re-parse to the exact same tree.
        assert_eq!(parse(&printed).unwrap(), parsed);
    }
}
