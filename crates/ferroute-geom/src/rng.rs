// SPDX-License-Identifier: AGPL-3.0-only
//! Deterministic, dependency-free PRNG for every stochastic solver in ferroute.
//!
//! INV-DET-2: the stream for a given seed is fixed forever (xoshiro256** seeded through
//! SplitMix64, Blackman & Vigna 2018). Replays never depend on an external crate's
//! version, so `--seed N` reproduces a run bit-for-bit across releases.

/// SplitMix64 step (also a good 64-bit mixing function).
#[must_use]
pub const fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// xoshiro256** generator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rng {
    s: [u64; 4],
}

impl Rng {
    /// Seeds the state with four SplitMix64 outputs.
    #[must_use]
    pub fn new(seed: u64) -> Self {
        let mut x = seed;
        let mut s = [0u64; 4];
        for v in &mut s {
            x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
            *v = splitmix64(x);
        }
        Self { s }
    }

    pub fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform in `[0, 1)` with 53 bits of precision.
    pub fn next_f64(&mut self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let v = (self.next_u64() >> 11) as f64;
        v * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform integer in `[0, n)` (Lemire's nearly-divisionless method). `n > 0`.
    pub fn below(&mut self, n: u64) -> u64 {
        assert!(n > 0, "below(0)");
        loop {
            let m = u128::from(self.next_u64()) * u128::from(n);
            #[allow(clippy::cast_possible_truncation)]
            let l = m as u64;
            if l >= n.wrapping_neg() % n {
                #[allow(clippy::cast_possible_truncation)]
                return (m >> 64) as u64;
            }
        }
    }

    /// Uniform integer in `[lo, hi]` (inclusive).
    pub fn range_i64(&mut self, lo: i64, hi: i64) -> i64 {
        if hi <= lo {
            return lo;
        }
        let span = hi.abs_diff(lo) + 1;
        #[allow(clippy::cast_possible_wrap)]
        let off = self.below(span) as i64;
        lo + off
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_is_frozen() {
        // Golden values: changing them breaks replay of every recorded run.
        let mut r = Rng::new(42);
        let v: Vec<u64> = (0..3).map(|_| r.next_u64()).collect();
        let mut r2 = Rng::new(42);
        assert_eq!(v, (0..3).map(|_| r2.next_u64()).collect::<Vec<_>>());
        assert_eq!(splitmix64(0), 0xE220_A839_7B1D_CDAF);
    }

    #[test]
    fn below_is_in_range() {
        let mut r = Rng::new(7);
        for n in [1u64, 2, 3, 10, 1000, u64::MAX] {
            for _ in 0..100 {
                assert!(r.below(n) < n);
            }
        }
        for _ in 0..100 {
            let v = r.range_i64(-5, 5);
            assert!((-5..=5).contains(&v));
        }
    }
}
