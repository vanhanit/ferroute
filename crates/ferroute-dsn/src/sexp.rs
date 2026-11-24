// SPDX-License-Identifier: AGPL-3.0-only
//! Minimal Specctra S-expression reader honouring `(string_quote X)`.

use crate::DsnError;

/// A parsed node.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Node {
    Atom(String),
    List(Vec<Node>),
}

impl Node {
    pub(crate) fn atom(&self) -> Option<&str> {
        match self {
            Self::Atom(s) => Some(s),
            Self::List(_) => None,
        }
    }

    pub(crate) fn items(&self) -> &[Self] {
        match self {
            Self::List(v) => v,
            Self::Atom(_) => &[],
        }
    }

    pub(crate) fn head(&self) -> Option<&str> {
        self.items().first().and_then(Self::atom)
    }

    /// First child list whose head is `name`.
    pub(crate) fn child<'a>(&'a self, name: &'a str) -> Option<&'a Self> {
        self.children(name).next()
    }

    pub(crate) fn children<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Self> + 'a {
        self.items().iter().filter(move |n| n.head() == Some(name))
    }
}

/// Finds the declared string quote (default `"`).
fn quote_char(src: &str) -> char {
    src.find("(string_quote")
        .and_then(|i| src[i + "(string_quote".len()..].chars().find(|c| !c.is_whitespace()))
        .filter(|c| *c != ')')
        .unwrap_or('"')
}

/// Parses `src` into its single top-level list.
pub(crate) fn parse(src: &str) -> Result<Node, DsnError> {
    let q = quote_char(src);
    let mut stack: Vec<Vec<Node>> = Vec::new();
    let mut root: Option<Node> = None;
    let mut chars = src.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '(' => stack.push(Vec::new()),
            ')' => {
                let done = stack.pop().ok_or_else(|| DsnError::Syntax("unbalanced ')'".into()))?;
                let node = Node::List(done);
                match stack.last_mut() {
                    Some(parent) => parent.push(node),
                    None if root.is_none() => root = Some(node),
                    None => return Err(DsnError::Syntax("trailing data after design".into())),
                }
            }
            c if c.is_whitespace() => {}
            c => {
                let mut s = String::new();
                let declares_quote = matches!(
                    stack.last().and_then(|l| l.last()),
                    Some(Node::Atom(a)) if a == "string_quote"
                );
                if declares_quote {
                    s.push(c);
                } else if c == q {
                    loop {
                        match chars.next() {
                            Some(x) if x == q => break,
                            Some(x) => s.push(x),
                            None => return Err(DsnError::Syntax("unterminated string".into())),
                        }
                    }
                } else {
                    s.push(c);
                    while let Some(&x) = chars.peek() {
                        if x.is_whitespace() || x == '(' || x == ')' {
                            break;
                        }
                        s.push(x);
                        chars.next();
                    }
                }
                stack
                    .last_mut()
                    .ok_or_else(|| DsnError::Syntax("atom outside a list".into()))?
                    .push(Node::Atom(s));
            }
        }
    }
    if !stack.is_empty() {
        return Err(DsnError::Syntax("unbalanced '('".into()));
    }
    root.ok_or_else(|| DsnError::Syntax("empty input".into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_and_quoted() {
        let n = parse("(pcb \"a b\" (x 1 2) (y))").unwrap();
        assert_eq!(n.items()[1].atom(), Some("a b"));
        assert_eq!(n.child("x").unwrap().items().len(), 3);
        assert!(n.child("y").is_some());
    }

    #[test]
    fn honours_custom_string_quote() {
        let n = parse("(pcb (parser (string_quote ')) (net 'a (b)'))").unwrap();
        assert_eq!(n.child("net").unwrap().items()[1].atom(), Some("a (b)"));
    }

    #[test]
    fn rejects_unbalanced() {
        assert!(parse("(a (b)").is_err());
        assert!(parse("(a))").is_err());
        assert!(parse("(a \"x)").is_err());
        assert!(parse("").is_err());
    }
}
