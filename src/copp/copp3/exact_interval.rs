//! Exact-sign kernel for one ordinary constant-jerk interval.
//!
//! These helpers classify whole-interval nonnegativity of `a(s)` from exact
//! dyadic signs of the supplied binary64 data, fix the polynomial anchor of a
//! delivered profile interval, and return exact logarithms on the cold paths
//! near endpoint and interior zeros. Profile time conversion and the
//! third-order planners share them, so this module must not depend on any
//! planner backend.

/// Number of 64-bit limbs needed to accumulate an exact orient2d determinant
/// of finite binary64 coordinates.
///
/// After factoring out 2^-2148, every product occupies at most 4196 bits and
/// summing seven such products needs fewer than 4199 bits.
const ORIENT2D_LIMBS: usize = 66;
/// Common power-of-two factor used by two-product orient/M accumulators.
const ORIENT2D_DYADIC_EXPONENT: i32 = -2148;
/// Number of limbs needed for the exact interval-interior expression
/// a*u - a*b - h*b*b.
///
/// After factoring out 2^-3222, each triple product occupies at most 6294 bits
/// and the three-term signed sum fits in 6296 bits.
const INTERVAL_D_LIMBS: usize = 99;
/// Common power-of-two factor used by IntervalDAccumulator.
const INTERVAL_D_DYADIC_EXPONENT: i32 = -3222;
/// Location of the minimum of the exact quadratic interval profile.
///
/// The branch is selected from the exact sign of `a + h*b` for the supplied
/// binary64 inputs.  In particular, a rounded signed zero does not select the
/// endpoint branch until its exact dyadic sign has been resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IntervalAMinimumBranch {
    Endpoint,
    Interior,
    Impossible,
}

/// Exact minimum-branch classification together with the rounded
/// \(M=a+h b\) reused by the filtered predicates.
#[derive(Clone, Copy)]
struct ClassifiedIntervalM {
    branch: IntervalAMinimumBranch,
    rounded: f64,
}

/// Exact nonnegative-domain classification for one ordinary interval.
///
/// The interior branch distinguishes a strictly positive minimum from an
/// exact internal double root; endpoint minima need no further subdivision.
#[derive(Clone, Copy, Debug)]
pub(crate) enum IntervalANonnegativeClassification {
    Endpoint,
    InteriorPositive,
    InteriorZero,
}

/// Accumulate the exact dyadic value \(M=a+h b\).
#[cold]
#[inline(never)]
fn exact_interval_m_slow(h: f64, a: f64, b: f64) -> Option<Orient2dAccumulator> {
    let mut exact = Orient2dAccumulator::new();
    (exact.add_product(a, 1.0, false) && exact.add_product(h, b, false)).then_some(exact)
}

/// Accumulate the exact right endpoint \(E=a+h b+h u\).
#[cold]
#[inline(never)]
fn exact_interval_endpoint_slow(h: f64, a: f64, b: f64, u: f64) -> Option<Orient2dAccumulator> {
    let mut exact = Orient2dAccumulator::new();
    (exact.add_product(a, 1.0, false)
        && exact.add_product(h, b, false)
        && exact.add_product(h, u, false))
    .then_some(exact)
}

/// Return the exact sign of \(M=a+h b\) on the cold path.
#[cold]
#[inline(never)]
fn exact_interval_m_sign_slow(h: f64, a: f64, b: f64) -> Option<std::cmp::Ordering> {
    exact_interval_m_slow(h, a, b).map(|exact| exact.sign())
}

/// Select the exact minimum branch and retain its fast rounded \(M\).
#[inline(always)]
fn classify_interval_m_valid(h: f64, a: f64, b: f64) -> Option<ClassifiedIntervalM> {
    let rounded = h.mul_add(b, a);
    let sign = if rounded > 0.0 {
        std::cmp::Ordering::Greater
    } else if rounded < 0.0 {
        std::cmp::Ordering::Less
    } else {
        exact_interval_m_sign_slow(h, a, b)?
    };
    let branch = if sign != std::cmp::Ordering::Less {
        IntervalAMinimumBranch::Endpoint
    } else if a > 0.0 {
        IntervalAMinimumBranch::Interior
    } else {
        IntervalAMinimumBranch::Impossible
    };
    Some(ClassifiedIntervalM { branch, rounded })
}

/// Return a one-ULP outward radius for a finite binary64 value.
#[inline(always)]
fn rounding_radius(value: f64) -> f64 {
    debug_assert!(value.is_finite());
    let magnitude = value.abs();
    magnitude.next_up() - magnitude
}

/// Add nonnegative error bounds and round the result upward.
#[inline(always)]
fn add_nonnegative_up(lhs: f64, rhs: f64) -> f64 {
    debug_assert!(lhs >= 0.0 && rhs >= 0.0);
    let sum = lhs + rhs;
    if sum.is_finite() {
        sum.next_up()
    } else {
        f64::INFINITY
    }
}

/// Multiply nonnegative error bounds and round the result upward.
#[inline(always)]
fn mul_nonnegative_up(lhs: f64, rhs: f64) -> f64 {
    debug_assert!(lhs >= 0.0 && rhs >= 0.0);
    if lhs == 0.0 || rhs == 0.0 {
        return 0.0;
    }
    let product = lhs * rhs;
    if product.is_finite() {
        product.next_up()
    } else {
        f64::INFINITY
    }
}

/// Return the exact sign of \(E=a+h b+h u\) on the cold path.
#[cold]
#[inline(never)]
fn exact_interval_endpoint_sign_slow(h: f64, a: f64, b: f64, u: f64) -> Option<std::cmp::Ordering> {
    exact_interval_endpoint_slow(h, a, b, u).map(|exact| exact.sign())
}

/// Accumulate the exact interior certificate
/// \(D=a u-a b-h b^2\).
#[cold]
#[inline(never)]
fn exact_interval_d_slow(h: f64, a: f64, b: f64, u: f64) -> Option<IntervalDAccumulator> {
    let mut exact = IntervalDAccumulator::new();
    (exact.add_product3(a, u, 1.0, false)
        && exact.add_product3(a, b, 1.0, true)
        && exact.add_product3(h, b, b, true))
    .then_some(exact)
}

/// Return the exact sign of the interior certificate \(D\).
#[cold]
#[inline(never)]
fn exact_interval_interior_sign_slow(h: f64, a: f64, b: f64, u: f64) -> Option<std::cmp::Ordering> {
    exact_interval_d_slow(h, a, b, u).map(|exact| exact.sign())
}

/// Exact sign of `E = a + h*b + h*u`, with a rigorously bounded FMA fast path.
#[inline(always)]
fn classify_interval_endpoint(
    h: f64,
    a: f64,
    b: f64,
    u: f64,
    rounded_m: f64,
) -> Option<std::cmp::Ordering> {
    if rounded_m.is_finite() {
        let rounded_e = h.mul_add(u, rounded_m);
        if rounded_e.is_finite() {
            // rounded_m = M + dm; rounded_e = rounded_m + h*u + de.
            // One upward ulp is a strict bound for each RN error, including
            // subnormal results.  The addition is rounded outwards as well.
            let error = add_nonnegative_up(rounding_radius(rounded_m), rounding_radius(rounded_e));
            if rounded_e > error {
                return Some(std::cmp::Ordering::Greater);
            }
            if rounded_e < -error {
                return Some(std::cmp::Ordering::Less);
            }
        }
    }
    exact_interval_endpoint_sign_slow(h, a, b, u)
}

/// Exact sign of `D = a*u - a*b - h*b*b`, with a rigorously bounded
/// multiply/FMA fast path.
#[inline(always)]
fn classify_interval_interior(
    h: f64,
    a: f64,
    b: f64,
    u: f64,
    rounded_m: f64,
) -> Option<std::cmp::Ordering> {
    if rounded_m.is_finite() {
        let rounded_au = a * u;
        let rounded_d = (-b).mul_add(rounded_m, rounded_au);
        if rounded_au.is_finite() && rounded_d.is_finite() {
            // rounded_d = D + d(au) - b*d(M) + d(fma).
            let error = add_nonnegative_up(
                add_nonnegative_up(
                    rounding_radius(rounded_au),
                    mul_nonnegative_up(b.abs(), rounding_radius(rounded_m)),
                ),
                rounding_radius(rounded_d),
            );
            if rounded_d > error {
                return Some(std::cmp::Ordering::Greater);
            }
            if rounded_d < -error {
                return Some(std::cmp::Ordering::Less);
            }
        }
    }
    exact_interval_interior_sign_slow(h, a, b, u)
}

/// Classify whole-interval nonnegativity after the exact \(M\)-branch is known.
#[inline(always)]
fn interval_a_nonnegative_classified(
    h: f64,
    a: f64,
    b: f64,
    u: f64,
    classified_m: ClassifiedIntervalM,
) -> Option<IntervalANonnegativeClassification> {
    match classified_m.branch {
        IntervalAMinimumBranch::Endpoint => (u >= 0.0
            || classify_interval_endpoint(h, a, b, u, classified_m.rounded)
                .is_some_and(|sign| sign != std::cmp::Ordering::Less))
        .then_some(IntervalANonnegativeClassification::Endpoint),
        IntervalAMinimumBranch::Interior => {
            if u <= 0.0 {
                return None;
            }
            let classified_d = classify_interval_interior(h, a, b, u, classified_m.rounded)?;
            match classified_d {
                std::cmp::Ordering::Greater => {
                    Some(IntervalANonnegativeClassification::InteriorPositive)
                }
                std::cmp::Ordering::Equal => Some(IntervalANonnegativeClassification::InteriorZero),
                std::cmp::Ordering::Less => None,
            }
        }
        IntervalAMinimumBranch::Impossible => None,
    }
}

/// Classify one ordinary constant-jerk interval with `a(s) >= 0` everywhere.
///
/// This uses exact signs and zero modelling tolerance for the supplied finite
/// `f64` data. `None` denotes an invalid or negative interval; equalities,
/// including `(a,b,u)=(0,0,0)`, are classified as nonnegative.
#[inline(always)]
pub(crate) fn classify_interval_a_nonnegative(
    h: f64,
    a: f64,
    b: f64,
    bnext: f64,
) -> Option<IntervalANonnegativeClassification> {
    if !h.is_finite()
        || h <= 0.0
        || !a.is_finite()
        || a < 0.0
        || !b.is_finite()
        || !bnext.is_finite()
    {
        return None;
    }
    let classified_m = classify_interval_m_valid(h, a, b)?;
    interval_a_nonnegative_classified(h, a, b, bnext, classified_m)
}

/// Natural logarithm of the exact positive dyadic value
/// `D = a*(u-b) - h*b*b`.
///
/// This is a cold-path companion to [`classify_interval_a_nonnegative`].  It
/// is intended for finite intervals so close to an interior double root that
/// forming `1-z` in binary64 loses the distance to the singularity.
#[cold]
#[inline(never)]
pub(crate) fn exact_interval_positive_d_ln(h: f64, a: f64, b: f64, u: f64) -> Option<f64> {
    if [h, a, b, u].into_iter().any(|value| !value.is_finite()) {
        return None;
    }
    let exact = exact_interval_d_slow(h, a, b, u)?;
    (exact.sign() == std::cmp::Ordering::Greater).then(|| exact.ln_abs(INTERVAL_D_DYADIC_EXPONENT))
}

/// Natural logarithm of the exact nonnegative dyadic value `M = a + h*b`.
/// Exact zero is returned as `-infinity`; a negative or non-finite value
/// returns `None`.
#[cold]
#[inline(never)]
pub(crate) fn exact_interval_nonnegative_m_ln(h: f64, a: f64, b: f64) -> Option<f64> {
    if [h, a, b].into_iter().any(|value| !value.is_finite()) {
        return None;
    }
    let exact = exact_interval_m_slow(h, a, b)?;
    match exact.sign() {
        std::cmp::Ordering::Greater => Some(exact.ln_abs(ORIENT2D_DYADIC_EXPONENT)),
        std::cmp::Ordering::Equal => Some(f64::NEG_INFINITY),
        std::cmp::Ordering::Less => None,
    }
}

/// Natural logarithm of `|M|` for an exact negative `M = a + h*b`.
#[cold]
#[inline(never)]
pub(crate) fn exact_interval_negative_m_abs_ln(h: f64, a: f64, b: f64) -> Option<f64> {
    if [h, a, b].into_iter().any(|value| !value.is_finite()) {
        return None;
    }
    let exact = exact_interval_m_slow(h, a, b)?;
    (exact.sign() == std::cmp::Ordering::Less).then(|| exact.ln_abs(ORIENT2D_DYADIC_EXPONENT))
}

/// Natural logarithm of the exact nonnegative right endpoint
/// `E = a + h*b + h*u`. Exact zero is returned as `-infinity`.
///
/// The rounded endpoint can underflow to zero even though this exact dyadic
/// sum is positive, so transition-time evaluation uses this cold helper to
/// distinguish a true zero from an unrepresentably small residual.
#[cold]
#[inline(never)]
fn exact_interval_nonnegative_endpoint_ln(h: f64, a: f64, b: f64, u: f64) -> Option<f64> {
    if [h, a, b, u].into_iter().any(|value| !value.is_finite()) {
        return None;
    }
    let exact = exact_interval_endpoint_slow(h, a, b, u)?;
    match exact.sign() {
        std::cmp::Ordering::Greater => Some(exact.ln_abs(ORIENT2D_DYADIC_EXPONENT)),
        std::cmp::Ordering::Equal => Some(f64::NEG_INFINITY),
        std::cmp::Ordering::Less => None,
    }
}

/// Certify the exact right endpoint from the two rounded FMAs used by the
/// transition kernel.
///
/// `Ok(None)` means the rounded positive endpoint is outside its strict error
/// envelope. `Ok(Some(ln_e))` is a cold exact result (`-infinity` denotes
/// exact zero), and `Err(())` denotes a negative or invalid exact endpoint.
#[inline(always)]
pub(crate) fn certify_interval_endpoint_ln(
    h: f64,
    a: f64,
    b: f64,
    u: f64,
    rounded_m: f64,
    rounded_e: f64,
) -> Result<Option<f64>, ()> {
    if !h.is_finite() || h <= 0.0 || !a.is_finite() || a < 0.0 || !b.is_finite() || !u.is_finite() {
        return Err(());
    }
    if rounded_m.is_finite() && rounded_e.is_finite() {
        let error = add_nonnegative_up(rounding_radius(rounded_m), rounding_radius(rounded_e));
        if rounded_e > error {
            return Ok(None);
        }
        if rounded_e < -error {
            return Err(());
        }
    }
    exact_interval_nonnegative_endpoint_ln(h, a, b, u)
        .map(Some)
        .ok_or(())
}

/// A delivered ordinary profile uses one deterministic polynomial anchor.
/// Preserve an exact zero left endpoint; otherwise a zero right endpoint or
/// the last ordinary edge is represented from the fixed right boundary.
/// This chooses a model before evaluating it, never the more permissive of
/// two SOC tests. The stored pair must come from the dynamics constructors.
#[inline]
pub(crate) fn profile_interval_uses_right_anchor(
    a_left: f64,
    a_right: f64,
    last_ordinary: bool,
) -> bool {
    a_left > 0.0 && (a_right == 0.0 || last_ordinary)
}

/// Signed fixed-size integer used only by rare exact dyadic-sign fallbacks.
#[derive(Clone, Copy)]
struct ExactDyadicAccumulator<const LIMBS: usize> {
    negative: bool,
    /// Half-open range of nonzero storage. `active_hi == 0` is canonical zero.
    active_lo: u16,
    active_hi: u16,
    magnitude: [u64; LIMBS],
}

/// Exact accumulator specialized for two-factor orientation predicates.
type Orient2dAccumulator = ExactDyadicAccumulator<ORIENT2D_LIMBS>;
/// Exact accumulator specialized for three-factor interval predicates.
type IntervalDAccumulator = ExactDyadicAccumulator<INTERVAL_D_LIMBS>;

impl<const LIMBS: usize> ExactDyadicAccumulator<LIMBS> {
    /// Construct the canonical exact zero.
    #[inline]
    fn new() -> Self {
        assert!(LIMBS <= u16::MAX as usize);
        Self {
            negative: false,
            active_lo: 0,
            active_hi: 0,
            magnitude: [0; LIMBS],
        }
    }

    /// Return whether the active limb range denotes canonical zero.
    #[inline(always)]
    fn is_zero(&self) -> bool {
        self.active_hi == 0
    }

    /// Return the first active limb index.
    #[inline(always)]
    fn active_lo(&self) -> usize {
        self.active_lo as usize
    }

    /// Return one past the last active limb index.
    #[inline(always)]
    fn active_hi(&self) -> usize {
        self.active_hi as usize
    }

    /// Store a nonempty active limb range.
    #[inline(always)]
    fn set_active_range(&mut self, lo: usize, hi: usize) {
        debug_assert!(lo < hi && hi <= LIMBS && hi <= u16::MAX as usize);
        self.active_lo = lo as u16;
        self.active_hi = hi as u16;
    }

    /// Trim zero boundary limbs and canonicalize an empty range.
    #[inline]
    fn normalize_active_range(&mut self, mut lo: usize, mut hi: usize) {
        while lo < hi && self.magnitude[lo] == 0 {
            lo += 1;
        }
        while lo < hi && self.magnitude[hi - 1] == 0 {
            hi -= 1;
        }
        if lo == hi {
            self.negative = false;
            self.active_lo = 0;
            self.active_hi = 0;
        } else {
            self.set_active_range(lo, hi);
        }
    }

    /// Compare the accumulator magnitude with a shifted compact term.
    #[inline]
    fn compare_compact_magnitude(
        &self,
        term: &[u64; 5],
        term_base: usize,
        term_lo: usize,
        term_hi: usize,
    ) -> std::cmp::Ordering {
        let term_global_lo = term_base + term_lo;
        let term_global_hi = term_base + term_hi;
        match self.active_hi().cmp(&term_global_hi) {
            std::cmp::Ordering::Equal => {}
            ordering => return ordering,
        }
        let lo = self.active_lo().min(term_global_lo);
        for id in (lo..term_global_hi).rev() {
            let rhs = if id >= term_global_lo {
                term[id - term_base]
            } else {
                0
            };
            match self.magnitude[id].cmp(&rhs) {
                std::cmp::Ordering::Equal => {}
                ordering => return ordering,
            }
        }
        std::cmp::Ordering::Equal
    }

    /// Add a shifted compact magnitude with carry propagation.
    #[inline]
    fn add_compact_magnitude(
        &mut self,
        term: &[u64; 5],
        term_base: usize,
        term_lo: usize,
        term_hi: usize,
    ) -> bool {
        let term_global_lo = term_base + term_lo;
        let term_global_hi = term_base + term_hi;
        let old_lo = self.active_lo();
        let old_hi = self.active_hi();
        let mut carry = false;
        for id in term_global_lo..term_global_hi {
            let (sum, carry_rhs) = self.magnitude[id].overflowing_add(term[id - term_base]);
            let (sum, carry_in) = sum.overflowing_add(u64::from(carry));
            self.magnitude[id] = sum;
            carry = carry_rhs || carry_in;
        }
        let mut id = term_global_hi;
        let mut result_hi = old_hi.max(term_global_hi);
        while carry {
            if id >= LIMBS {
                return false;
            }
            let (sum, next_carry) = self.magnitude[id].overflowing_add(1);
            self.magnitude[id] = sum;
            id += 1;
            result_hi = result_hi.max(id);
            carry = next_carry;
        }
        self.normalize_active_range(old_lo.min(term_global_lo), result_hi);
        true
    }

    /// Subtract the compact term, whose magnitude is strictly smaller.
    #[inline]
    fn subtract_compact_magnitude(
        &mut self,
        term: &[u64; 5],
        term_base: usize,
        term_lo: usize,
        term_hi: usize,
    ) {
        let term_global_lo = term_base + term_lo;
        let term_global_hi = term_base + term_hi;
        let old_lo = self.active_lo();
        let old_hi = self.active_hi();
        let mut borrow = false;
        for id in term_global_lo..term_global_hi {
            let (difference, borrow_rhs) = self.magnitude[id].overflowing_sub(term[id - term_base]);
            let (difference, borrow_in) = difference.overflowing_sub(u64::from(borrow));
            self.magnitude[id] = difference;
            borrow = borrow_rhs || borrow_in;
        }
        let mut id = term_global_hi;
        while borrow {
            debug_assert!(id < old_hi);
            let (difference, next_borrow) = self.magnitude[id].overflowing_sub(1);
            self.magnitude[id] = difference;
            id += 1;
            borrow = next_borrow;
        }
        debug_assert!(!borrow);
        self.normalize_active_range(old_lo.min(term_global_lo), old_hi);
    }

    /// Replace the accumulator by `term - abs(self)`, known to be positive.
    #[inline]
    fn replace_with_compact_difference(
        &mut self,
        negative: bool,
        term: &[u64; 5],
        term_base: usize,
        term_lo: usize,
        term_hi: usize,
    ) {
        let term_global_lo = term_base + term_lo;
        let term_global_hi = term_base + term_hi;
        let old_lo = self.active_lo();
        let old_hi = self.active_hi();
        debug_assert!(term_global_hi >= old_hi);
        let lo = old_lo.min(term_global_lo);
        let mut borrow = false;
        for id in lo..term_global_hi {
            let term_word = if id >= term_global_lo {
                term[id - term_base]
            } else {
                0
            };
            let old_word = self.magnitude[id];
            let (difference, borrow_rhs) = term_word.overflowing_sub(old_word);
            let (difference, borrow_in) = difference.overflowing_sub(u64::from(borrow));
            self.magnitude[id] = difference;
            borrow = borrow_rhs || borrow_in;
        }
        debug_assert!(!borrow);
        self.negative = negative;
        self.normalize_active_range(lo, term_global_hi);
    }

    /// Add a little-endian unsigned word sequence times `2^shift` exactly.
    fn add_words<const WORDS: usize>(
        &mut self,
        negative: bool,
        words: [u64; WORDS],
        shift: usize,
    ) -> bool {
        debug_assert!(WORDS <= 4);
        if WORDS > 4 {
            return false;
        }
        let mut term = [0_u64; 5];
        let term_base = shift / 64;
        let bit = shift % 64;
        for (id_word, word) in words.into_iter().enumerate() {
            term[id_word] |= word << bit;
            if bit != 0 {
                term[id_word + 1] |= word >> (64 - bit);
            }
        }
        let Some(term_lo) = term.iter().position(|&word| word != 0) else {
            return true;
        };
        let term_hi = term.iter().rposition(|&word| word != 0).unwrap() + 1;
        let Some(term_global_hi) = term_base.checked_add(term_hi) else {
            return false;
        };
        if term_global_hi > LIMBS {
            return false;
        }
        let term_global_lo = term_base + term_lo;

        if self.is_zero() {
            self.negative = negative;
            self.magnitude[term_global_lo..term_global_hi].copy_from_slice(&term[term_lo..term_hi]);
            self.set_active_range(term_global_lo, term_global_hi);
            return true;
        }
        if self.negative == negative {
            return self.add_compact_magnitude(&term, term_base, term_lo, term_hi);
        }

        match self.compare_compact_magnitude(&term, term_base, term_lo, term_hi) {
            std::cmp::Ordering::Greater => {
                self.subtract_compact_magnitude(&term, term_base, term_lo, term_hi)
            }
            std::cmp::Ordering::Equal => {
                let lo = self.active_lo();
                let hi = self.active_hi();
                self.magnitude[lo..hi].fill(0);
                self.negative = false;
                self.active_lo = 0;
                self.active_hi = 0;
            }
            std::cmp::Ordering::Less => {
                self.replace_with_compact_difference(negative, &term, term_base, term_lo, term_hi);
            }
        }
        true
    }

    /// Add `(+/- mantissa) * 2^shift` exactly.
    #[inline]
    fn add_term(&mut self, negative: bool, mantissa: u128, shift: usize) -> bool {
        self.add_words(negative, [mantissa as u64, (mantissa >> 64) as u64], shift)
    }

    /// Add or subtract the product of two already decomposed finite values.
    #[inline(always)]
    fn add_decomposed_product(
        &mut self,
        lhs: (bool, u64, usize),
        rhs: (bool, u64, usize),
        subtract: bool,
    ) -> bool {
        let (lhs_negative, lhs_mantissa, lhs_shift) = lhs;
        let (rhs_negative, rhs_mantissa, rhs_shift) = rhs;
        self.add_term(
            lhs_negative ^ rhs_negative ^ subtract,
            (lhs_mantissa as u128) * (rhs_mantissa as u128),
            lhs_shift + rhs_shift,
        )
    }

    /// Add or subtract one exact binary64 product.
    fn add_product(&mut self, lhs: f64, rhs: f64, subtract: bool) -> bool {
        self.add_decomposed_product(finite_f64_dyadic(lhs), finite_f64_dyadic(rhs), subtract)
    }

    /// Add or subtract one exact product of three binary64 factors.
    fn add_product3(&mut self, lhs: f64, middle: f64, rhs: f64, subtract: bool) -> bool {
        let (lhs_negative, lhs_mantissa, lhs_shift) = finite_f64_dyadic(lhs);
        let (middle_negative, middle_mantissa, middle_shift) = finite_f64_dyadic(middle);
        let (rhs_negative, rhs_mantissa, rhs_shift) = finite_f64_dyadic(rhs);

        // Convolve the 106-bit lhs*middle product with the 53-bit rhs
        // mantissa. The three words represent the integer coefficient before
        // the shared power-of-two shift is applied.
        let lhs_middle = (lhs_mantissa as u128) * (middle_mantissa as u128);
        let low_product = (lhs_middle as u64 as u128) * (rhs_mantissa as u128);
        let high_product = (lhs_middle >> 64) * (rhs_mantissa as u128);
        let middle_sum = (low_product >> 64) + (high_product as u64 as u128);
        let words = [
            low_product as u64,
            middle_sum as u64,
            ((high_product >> 64) + (middle_sum >> 64)) as u64,
        ];
        self.add_words(
            lhs_negative ^ middle_negative ^ rhs_negative ^ subtract,
            words,
            lhs_shift + middle_shift + rhs_shift,
        )
    }

    /// Return the exact sign encoded by the sign bit and active magnitude.
    #[inline]
    fn sign(&self) -> std::cmp::Ordering {
        if self.is_zero() {
            std::cmp::Ordering::Equal
        } else if self.negative {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        }
    }

    /// Approximate `ln(abs(self) * 2^dyadic_exponent)` from the leading
    /// limbs.  The accumulator itself remains exact; only the final logarithm
    /// is rounded to binary64.  One following limb supplies guard bits without
    /// allocation or conversion through a big-integer type.
    fn ln_abs(&self, dyadic_exponent: i32) -> f64 {
        debug_assert!(!self.is_zero());
        let id_high = self
            .magnitude
            .iter()
            .rposition(|&limb| limb != 0)
            .expect("nonzero exact dyadic accumulator");
        let high = self.magnitude[id_high];
        let id_bit = 63 - high.leading_zeros() as usize;
        const TWO_NEG_64: f64 = 5.421_010_862_427_522e-20;
        let following = id_high
            .checked_sub(1)
            .map_or(0.0, |id| self.magnitude[id] as f64 * TWO_NEG_64);
        let normalized = (high as f64 + following) * 2.0_f64.powi(-(id_bit as i32));
        let exponent = dyadic_exponent + (64 * id_high + id_bit) as i32;
        normalized.ln() + f64::from(exponent) * std::f64::consts::LN_2
    }
}

/// Return `(negative, integer mantissa, power-of-two shift)` relative to the
/// common factor `2^-1074`. The caller has already rejected non-finite values.
#[inline]
fn finite_f64_dyadic(value: f64) -> (bool, u64, usize) {
    let bits = value.to_bits();
    let exponent = ((bits >> 52) & 0x7ff) as usize;
    let fraction = bits & ((1_u64 << 52) - 1);
    let mantissa = if exponent == 0 {
        fraction
    } else {
        (1_u64 << 52) | fraction
    };
    (bits >> 63 != 0, mantissa, exponent.saturating_sub(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Independent copy of the pre-sparse exact accumulator.  Differential
    /// tests intentionally share neither active-range bookkeeping nor compact
    /// term arithmetic with the production implementation.
    struct DenseExactDyadicAccumulator<const LIMBS: usize> {
        negative: bool,
        magnitude: [u64; LIMBS],
    }

    impl<const LIMBS: usize> DenseExactDyadicAccumulator<LIMBS> {
        fn new() -> Self {
            Self {
                negative: false,
                magnitude: [0; LIMBS],
            }
        }

        fn is_zero(&self) -> bool {
            self.magnitude.iter().all(|&limb| limb == 0)
        }

        fn compare_magnitude(lhs: &[u64; LIMBS], rhs: &[u64; LIMBS]) -> std::cmp::Ordering {
            for (&lhs_limb, &rhs_limb) in lhs.iter().zip(rhs.iter()).rev() {
                match lhs_limb.cmp(&rhs_limb) {
                    std::cmp::Ordering::Equal => {}
                    ordering => return ordering,
                }
            }
            std::cmp::Ordering::Equal
        }

        fn add_magnitude(&mut self, rhs: &[u64; LIMBS]) -> bool {
            let mut carry = false;
            for (lhs_limb, &rhs_limb) in self.magnitude.iter_mut().zip(rhs.iter()) {
                let (sum, carry_rhs) = lhs_limb.overflowing_add(rhs_limb);
                let (sum, carry_in) = sum.overflowing_add(u64::from(carry));
                *lhs_limb = sum;
                carry = carry_rhs || carry_in;
            }
            !carry
        }

        fn subtract_magnitude(&mut self, rhs: &[u64; LIMBS]) {
            let mut borrow = false;
            for (lhs_limb, &rhs_limb) in self.magnitude.iter_mut().zip(rhs.iter()) {
                let (difference, borrow_rhs) = lhs_limb.overflowing_sub(rhs_limb);
                let (difference, borrow_in) = difference.overflowing_sub(u64::from(borrow));
                *lhs_limb = difference;
                borrow = borrow_rhs || borrow_in;
            }
            debug_assert!(!borrow);
        }

        fn add_words<const WORDS: usize>(
            &mut self,
            negative: bool,
            words: [u64; WORDS],
            shift: usize,
        ) -> bool {
            if words.iter().all(|&word| word == 0) {
                return true;
            }

            let mut term = [0_u64; LIMBS];
            let limb = shift / 64;
            let bit = shift % 64;
            for (id_word, word) in words.into_iter().enumerate() {
                if word == 0 {
                    continue;
                }
                let id_limb = limb + id_word;
                let low = word << bit;
                if low != 0 {
                    let Some(slot) = term.get_mut(id_limb) else {
                        return false;
                    };
                    *slot |= low;
                }
                if bit != 0 {
                    let high = word >> (64 - bit);
                    if high != 0 {
                        let Some(slot) = term.get_mut(id_limb + 1) else {
                            return false;
                        };
                        *slot |= high;
                    }
                }
            }

            if self.is_zero() {
                self.negative = negative;
                self.magnitude = term;
                return true;
            }
            if self.negative == negative {
                return self.add_magnitude(&term);
            }

            match Self::compare_magnitude(&self.magnitude, &term) {
                std::cmp::Ordering::Greater => self.subtract_magnitude(&term),
                std::cmp::Ordering::Equal => {
                    self.magnitude.fill(0);
                    self.negative = false;
                }
                std::cmp::Ordering::Less => {
                    let old_magnitude = self.magnitude;
                    self.magnitude = term;
                    self.subtract_magnitude(&old_magnitude);
                    self.negative = negative;
                }
            }
            true
        }

        fn add_term(&mut self, negative: bool, mantissa: u128, shift: usize) -> bool {
            self.add_words(negative, [mantissa as u64, (mantissa >> 64) as u64], shift)
        }
    }

    impl<const LIMBS: usize> DenseExactDyadicAccumulator<LIMBS> {
        fn add_product(&mut self, lhs: f64, rhs: f64, subtract: bool) -> bool {
            let (lhs_negative, lhs_mantissa, lhs_shift) = dense_finite_f64_dyadic(lhs);
            let (rhs_negative, rhs_mantissa, rhs_shift) = dense_finite_f64_dyadic(rhs);
            self.add_term(
                lhs_negative ^ rhs_negative ^ subtract,
                (lhs_mantissa as u128) * (rhs_mantissa as u128),
                lhs_shift + rhs_shift,
            )
        }

        fn add_product3(&mut self, lhs: f64, middle: f64, rhs: f64, subtract: bool) -> bool {
            let (lhs_negative, lhs_mantissa, lhs_shift) = dense_finite_f64_dyadic(lhs);
            let (middle_negative, middle_mantissa, middle_shift) = dense_finite_f64_dyadic(middle);
            let (rhs_negative, rhs_mantissa, rhs_shift) = dense_finite_f64_dyadic(rhs);
            let lhs_middle = (lhs_mantissa as u128) * (middle_mantissa as u128);
            let low_product = (lhs_middle as u64 as u128) * (rhs_mantissa as u128);
            let high_product = (lhs_middle >> 64) * (rhs_mantissa as u128);
            let middle_sum = (low_product >> 64) + (high_product as u64 as u128);
            self.add_words(
                lhs_negative ^ middle_negative ^ rhs_negative ^ subtract,
                [
                    low_product as u64,
                    middle_sum as u64,
                    ((high_product >> 64) + (middle_sum >> 64)) as u64,
                ],
                lhs_shift + middle_shift + rhs_shift,
            )
        }

        fn sign(&self) -> std::cmp::Ordering {
            if self.is_zero() {
                std::cmp::Ordering::Equal
            } else if self.negative {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        }
    }

    fn dense_finite_f64_dyadic(value: f64) -> (bool, u64, usize) {
        let bits = value.to_bits();
        let exponent = ((bits >> 52) & 0x7ff) as usize;
        let fraction = bits & ((1_u64 << 52) - 1);
        let mantissa = if exponent == 0 {
            fraction
        } else {
            (1_u64 << 52) | fraction
        };
        (bits >> 63 != 0, mantissa, exponent.saturating_sub(1))
    }

    impl<const LIMBS: usize> DenseExactDyadicAccumulator<LIMBS> {
        fn ln_abs(&self, dyadic_exponent: i32) -> f64 {
            debug_assert!(!self.is_zero());
            let id_high = self.magnitude.iter().rposition(|&x| x != 0).unwrap();
            let high = self.magnitude[id_high];
            let id_bit = 63 - high.leading_zeros() as usize;
            let following = id_high.checked_sub(1).map_or(0.0, |id| {
                self.magnitude[id] as f64 * 5.421_010_862_427_522e-20
            });
            let normalized = (high as f64 + following) * 2.0_f64.powi(-(id_bit as i32));
            let exponent = dyadic_exponent + (64 * id_high + id_bit) as i32;
            normalized.ln() + f64::from(exponent) * std::f64::consts::LN_2
        }
    }

    fn assert_sparse_dense_same<const LIMBS: usize>(
        sparse: &ExactDyadicAccumulator<LIMBS>,
        dense: &DenseExactDyadicAccumulator<LIMBS>,
        dyadic_exponent: i32,
    ) {
        assert_eq!(sparse.negative, dense.negative);
        assert_eq!(sparse.magnitude, dense.magnitude);
        assert_eq!(sparse.is_zero(), dense.is_zero());
        assert_eq!(sparse.sign(), dense.sign());
        let expected_lo = dense.magnitude.iter().position(|&word| word != 0);
        let expected_hi = dense.magnitude.iter().rposition(|&word| word != 0);
        match (expected_lo, expected_hi) {
            (None, None) => {
                assert_eq!((sparse.active_lo, sparse.active_hi), (0, 0));
                assert!(!sparse.negative);
            }
            (Some(lo), Some(hi)) => {
                assert_eq!(sparse.active_lo(), lo);
                assert_eq!(sparse.active_hi(), hi + 1);
                assert!(sparse.magnitude[..lo].iter().all(|&word| word == 0));
                assert!(sparse.magnitude[hi + 1..].iter().all(|&word| word == 0));
                assert_eq!(
                    sparse.ln_abs(dyadic_exponent).to_bits(),
                    dense.ln_abs(dyadic_exponent).to_bits(),
                );
            }
            _ => unreachable!(),
        }
    }

    fn dyadic_test_splitmix64(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut value = *state;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    fn dyadic_test_finite(state: &mut u64) -> f64 {
        let mut bits = dyadic_test_splitmix64(state);
        if bits & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000 {
            bits ^= 0x0010_0000_0000_0000;
        }
        f64::from_bits(bits)
    }

    fn dyadic_test_factor(state: &mut u64, step: usize) -> f64 {
        const EDGES: [f64; 16] = [
            0.0,
            -0.0,
            f64::from_bits(1),
            -f64::from_bits(1),
            f64::MIN_POSITIVE.next_down(),
            -f64::MIN_POSITIVE.next_down(),
            f64::MIN_POSITIVE,
            -f64::MIN_POSITIVE,
            0.5,
            -0.5,
            1.0,
            -1.0,
            f64::from_bits(0x7fe0_0000_0000_0000),
            -f64::from_bits(0x7fe0_0000_0000_0000),
            f64::MAX,
            -f64::MAX,
        ];
        if step.is_multiple_of(4) {
            EDGES[(dyadic_test_splitmix64(state) as usize) % EDGES.len()]
        } else {
            dyadic_test_finite(state)
        }
    }

    fn differential_add_words<const LIMBS: usize, const WORDS: usize>(
        sparse: &mut ExactDyadicAccumulator<LIMBS>,
        dense: &mut DenseExactDyadicAccumulator<LIMBS>,
        negative: bool,
        words: [u64; WORDS],
        shift: usize,
        dyadic_exponent: i32,
    ) -> bool {
        let sparse_ok = sparse.add_words(negative, words, shift);
        let dense_ok = dense.add_words(negative, words, shift);
        assert_eq!(
            sparse_ok, dense_ok,
            "capacity result differs at shift {shift}"
        );
        if sparse_ok {
            assert_sparse_dense_same(sparse, dense, dyadic_exponent);
        }
        sparse_ok
    }

    fn exercise_exact_dyadic_word_branches<const LIMBS: usize>(dyadic_exponent: i32) {
        assert!(LIMBS >= 5);

        let mut sparse = ExactDyadicAccumulator::<LIMBS>::new();
        let mut dense = DenseExactDyadicAccumulator::<LIMBS>::new();
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [0_u64; 4],
            usize::MAX,
            dyadic_exponent,
        ));

        // Same-sign carry can erase both low limbs and move the active low end.
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [u64::MAX, u64::MAX],
            0,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            0,
            dyadic_exponent,
        ));
        assert_eq!((sparse.active_lo(), sparse.active_hi()), (2, 3));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            128,
            dyadic_exponent,
        ));
        assert!(sparse.is_zero());

        // A borrow crosses three zero limbs, lowering both active endpoints.
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            192,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            0,
            dyadic_exponent,
        ));
        assert_eq!((sparse.active_lo(), sparse.active_hi()), (0, 3));
        assert!(sparse.magnitude[..3].iter().all(|&word| word == u64::MAX));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            0,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            192,
            dyadic_exponent,
        ));
        assert!(sparse.is_zero());

        // A larger opposite-sign compact term replaces the old magnitude.
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            0,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            192,
            dyadic_exponent,
        ));
        assert!(sparse.negative);
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [u64::MAX, u64::MAX, u64::MAX],
            0,
            dyadic_exponent,
        ));
        assert!(sparse.is_zero());

        // `term - self` may cancel the term's high limb and shrink to one.
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [u64::MAX, u64::MAX],
            0,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            128,
            dyadic_exponent,
        ));
        assert!(sparse.negative);
        assert_eq!((sparse.active_lo(), sparse.active_hi()), (0, 1));
        assert_eq!(sparse.magnitude[0], 1);
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            0,
            dyadic_exponent,
        ));

        // Equal integers with different compact base/leading-zero layouts cancel.
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            64,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [0_u64, 1],
            0,
            dyadic_exponent,
        ));
        assert!(sparse.is_zero());

        // A non-aligned four-word term exercises the fifth compact limb.
        let wide = [1_u64, 2, 3, 1_u64 << 63];
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            wide,
            63,
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            wide,
            63,
            dyadic_exponent,
        ));
        assert!(sparse.is_zero());

        // The final storage limb is usable, but any real bit beyond it is not.
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            64 * (LIMBS - 1),
            dyadic_exponent,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            64 * (LIMBS - 1),
            dyadic_exponent,
        ));

        let mut overflow_sparse = ExactDyadicAccumulator::<LIMBS>::new();
        let mut overflow_dense = DenseExactDyadicAccumulator::<LIMBS>::new();
        assert!(differential_add_words(
            &mut overflow_sparse,
            &mut overflow_dense,
            false,
            [u64::MAX, 0],
            64 * (LIMBS - 1),
            dyadic_exponent,
        ));
        // Failure is deliberately non-transactional; discard both objects.
        assert!(!differential_add_words(
            &mut overflow_sparse,
            &mut overflow_dense,
            false,
            [1_u64, 0],
            64 * (LIMBS - 1),
            dyadic_exponent,
        ));

        for (words, shift) in [([1_u64], 64 * LIMBS), ([3_u64], 64 * LIMBS - 1)] {
            let mut sparse = ExactDyadicAccumulator::<LIMBS>::new();
            let mut dense = DenseExactDyadicAccumulator::<LIMBS>::new();
            assert!(!differential_add_words(
                &mut sparse,
                &mut dense,
                false,
                words,
                shift,
                dyadic_exponent,
            ));
        }
    }

    #[test]
    fn sparse_exact_dyadic_word_arithmetic_matches_dense_oracle() {
        assert_eq!(
            std::mem::size_of::<ExactDyadicAccumulator<ORIENT2D_LIMBS>>(),
            std::mem::size_of::<DenseExactDyadicAccumulator<ORIENT2D_LIMBS>>()
        );
        assert_eq!(
            std::mem::size_of::<ExactDyadicAccumulator<INTERVAL_D_LIMBS>>(),
            std::mem::size_of::<DenseExactDyadicAccumulator<INTERVAL_D_LIMBS>>()
        );
        exercise_exact_dyadic_word_branches::<ORIENT2D_LIMBS>(ORIENT2D_DYADIC_EXPONENT);
        exercise_exact_dyadic_word_branches::<INTERVAL_D_LIMBS>(INTERVAL_D_DYADIC_EXPONENT);
    }

    fn exercise_exact_dyadic_products2() {
        for case in 0_u64..16 {
            let mut state = 0x3d91_4e2b_7a65_c8f0 ^ case.wrapping_mul(0x517c_c1b7_2722_0a95);
            let mut sparse = Orient2dAccumulator::new();
            let mut dense = DenseExactDyadicAccumulator::<ORIENT2D_LIMBS>::new();
            for step in 0..96 {
                let lhs = dyadic_test_factor(&mut state, 2 * step);
                let rhs = dyadic_test_factor(&mut state, 2 * step + 1);
                let subtract = dyadic_test_splitmix64(&mut state) & 1 != 0;
                let sparse_ok = sparse.add_product(lhs, rhs, subtract);
                let dense_ok = dense.add_product(lhs, rhs, subtract);
                assert_eq!(sparse_ok, dense_ok, "pair capacity mismatch: {case}/{step}");
                assert!(sparse_ok, "66 limbs must hold 96 finite pair products");
                assert_sparse_dense_same(&sparse, &dense, ORIENT2D_DYADIC_EXPONENT);

                if step % 13 == 0 {
                    assert!(sparse.add_product(lhs, rhs, !subtract));
                    assert!(dense.add_product(lhs, rhs, !subtract));
                    assert_sparse_dense_same(&sparse, &dense, ORIENT2D_DYADIC_EXPONENT);
                }
                if step == 47 {
                    let mut branch_sparse = sparse;
                    let mut branch_dense = DenseExactDyadicAccumulator {
                        negative: dense.negative,
                        magnitude: dense.magnitude,
                    };
                    assert!(branch_sparse.add_product(-0.5, f64::MAX, false));
                    assert!(branch_dense.add_product(-0.5, f64::MAX, false));
                    assert_sparse_dense_same(
                        &branch_sparse,
                        &branch_dense,
                        ORIENT2D_DYADIC_EXPONENT,
                    );
                }
            }
        }
    }

    fn exercise_exact_dyadic_products3() {
        for case in 0_u64..16 {
            let mut state = 0xb024_a83f_165e_7cd9 ^ case.wrapping_mul(0x94d0_49bb_1331_11eb);
            let mut sparse = IntervalDAccumulator::new();
            let mut dense = DenseExactDyadicAccumulator::<INTERVAL_D_LIMBS>::new();
            for step in 0..96 {
                let lhs = dyadic_test_factor(&mut state, 3 * step);
                let middle = dyadic_test_factor(&mut state, 3 * step + 1);
                let rhs = dyadic_test_factor(&mut state, 3 * step + 2);
                let subtract = dyadic_test_splitmix64(&mut state) & 1 != 0;
                let sparse_ok = sparse.add_product3(lhs, middle, rhs, subtract);
                let dense_ok = dense.add_product3(lhs, middle, rhs, subtract);
                assert_eq!(
                    sparse_ok, dense_ok,
                    "triple capacity mismatch: {case}/{step}"
                );
                assert!(sparse_ok, "99 limbs must hold 96 finite triple products");
                assert_sparse_dense_same(&sparse, &dense, INTERVAL_D_DYADIC_EXPONENT);

                if step % 13 == 0 {
                    assert!(sparse.add_product3(lhs, middle, rhs, !subtract));
                    assert!(dense.add_product3(lhs, middle, rhs, !subtract));
                    assert_sparse_dense_same(&sparse, &dense, INTERVAL_D_DYADIC_EXPONENT);
                }
            }
        }
    }

    #[test]
    fn sparse_exact_dyadic_products_match_dense_oracle() {
        exercise_exact_dyadic_products2();
        exercise_exact_dyadic_products3();
    }

    #[test]
    fn exact_dyadic_design_capacities_hold_maximum_products() {
        let mut pair = Orient2dAccumulator::new();
        let mut dense_pair = DenseExactDyadicAccumulator::<ORIENT2D_LIMBS>::new();
        for _ in 0..7 {
            assert!(pair.add_product(f64::MAX, f64::MAX, false));
            assert!(dense_pair.add_product(f64::MAX, f64::MAX, false));
            assert_sparse_dense_same(&pair, &dense_pair, ORIENT2D_DYADIC_EXPONENT);
        }
        for _ in 0..7 {
            assert!(pair.add_product(f64::MAX, f64::MAX, true));
            assert!(dense_pair.add_product(f64::MAX, f64::MAX, true));
            assert_sparse_dense_same(&pair, &dense_pair, ORIENT2D_DYADIC_EXPONENT);
        }
        assert!(pair.is_zero());

        let mut triple = IntervalDAccumulator::new();
        let mut dense_triple = DenseExactDyadicAccumulator::<INTERVAL_D_LIMBS>::new();
        for _ in 0..3 {
            assert!(triple.add_product3(f64::MAX, f64::MAX, f64::MAX, false));
            assert!(dense_triple.add_product3(f64::MAX, f64::MAX, f64::MAX, false));
            assert_sparse_dense_same(&triple, &dense_triple, INTERVAL_D_DYADIC_EXPONENT);
        }
        for _ in 0..3 {
            assert!(triple.add_product3(f64::MAX, f64::MAX, f64::MAX, true));
            assert!(dense_triple.add_product3(f64::MAX, f64::MAX, f64::MAX, true));
            assert_sparse_dense_same(&triple, &dense_triple, INTERVAL_D_DYADIC_EXPONENT);
        }
        assert!(triple.is_zero());
    }

    #[test]
    fn exact_dyadic_ln_tracks_high_limb_and_high_cancellation() {
        for shift in [
            0,
            64 * (INTERVAL_D_LIMBS / 2),
            64 * (INTERVAL_D_LIMBS - 1) + 63,
        ] {
            let mut sparse = IntervalDAccumulator::new();
            let mut dense = DenseExactDyadicAccumulator::<INTERVAL_D_LIMBS>::new();
            assert!(differential_add_words(
                &mut sparse,
                &mut dense,
                true,
                [1_u64],
                shift,
                -(shift as i32),
            ));
            assert_eq!(sparse.ln_abs(-(shift as i32)).to_bits(), 0.0_f64.to_bits());
        }

        let high_shift = 64 * (INTERVAL_D_LIMBS - 2);
        let mut sparse = IntervalDAccumulator::new();
        let mut dense = DenseExactDyadicAccumulator::<INTERVAL_D_LIMBS>::new();
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            high_shift,
            0,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [u64::MAX],
            high_shift - 64,
            0,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            false,
            [1_u64],
            0,
            0,
        ));
        assert!(differential_add_words(
            &mut sparse,
            &mut dense,
            true,
            [1_u64],
            high_shift,
            0,
        ));
        assert_eq!(
            (sparse.active_lo(), sparse.active_hi()),
            (0, INTERVAL_D_LIMBS - 2)
        );
    }

    #[test]
    fn exact_interval_classifies_internal_double_root_neighbors() {
        let h = 1.0_f64;
        let a = 2.0_f64.powi(-15);
        let b = -2.0_f64.powi(-11);
        let u_equal = 15.0 * 2.0_f64.powi(-11);
        let u_minus = u_equal.next_down();
        let u_plus = u_equal.next_up();

        assert!(classify_interval_a_nonnegative(h, a, b, u_minus).is_none());
        assert!(matches!(
            classify_interval_a_nonnegative(h, a, b, u_equal),
            Some(IntervalANonnegativeClassification::InteriorZero)
        ));
        assert!(matches!(
            classify_interval_a_nonnegative(h, a, b, u_plus),
            Some(IntervalANonnegativeClassification::InteriorPositive)
        ));

        let ln_d = exact_interval_positive_d_ln(h, a, b, u_plus).unwrap();
        let expected = -75.0 * std::f64::consts::LN_2;
        assert!((ln_d - expected).abs() <= 8.0 * f64::EPSILON * expected.abs());
    }
}
