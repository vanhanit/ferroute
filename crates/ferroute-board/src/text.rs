// SPDX-License-Identifier: AGPL-3.0-only
//! Small text helpers shared by every crate (single implementation, DRY).

/// Glob match supporting `*` (any run) and `?` (one character); case-sensitive like
/// KiCad references and net names.
#[must_use]
pub fn glob(pattern: &str, text: &str) -> bool {
    let p: Vec<char> = pattern.chars().collect();
    let t: Vec<char> = text.chars().collect();
    let (mut pi, mut ti) = (0usize, 0usize);
    let mut star: Option<(usize, usize)> = None;
    while ti < t.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == t[ti]) {
            pi += 1;
            ti += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some((pi, ti));
            pi += 1;
        } else if let Some((sp, st)) = star {
            pi = sp + 1;
            ti = st + 1;
            star = Some((sp, st + 1));
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

#[cfg(test)]
mod tests {
    use super::glob;

    #[test]
    fn globbing() {
        assert!(glob("*", "anything"));
        assert!(glob("*", ""));
        assert!(glob("+3V*", "+3V3"));
        assert!(glob("R*", "R12"));
        assert!(glob("D?", "D1"));
        assert!(!glob("D?", "D12"));
        assert!(glob("*GND*", "AGND_SENSE"));
        assert!(!glob("GND", "GNDA"));
        assert!(glob("a*b*c", "aXXbYYc"));
        assert!(!glob("a*b*c", "aXXbYY"));
        assert!(glob("U*1", "U101"));
        assert!(!glob("C*", "R1"));
    }
}
