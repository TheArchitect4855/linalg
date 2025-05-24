const std = @import("std");

/// Utility functions for vector types.
pub fn Vectors(N: type) type {
    const epsilon = std.math.floatEps(N) * 5.0;
    return struct {
        // --- CONSTANTS ---

        /// A vector with the Z component equal to -1.
        pub fn back(n: comptime_int) @Vector(n, N) {
            var v = zero(n);
            v[2] = -1.0;
        }

        /// A vector with the Y component equal to -1.
        pub fn down(n: comptime_int) @Vector(n, N) {
            var v = zero(n);
            v[1] = -1.0;
            return v;
        }

        /// A vector with the Z component equal to 1.
        pub fn forward(n: comptime_int) @Vector(n, N) {
            var v = zero(n);
            v[2] = 1.0;
            return v;
        }

        /// A vector with all components set to infinity.
        pub fn inf(n: comptime_int) @Vector(n, N) {
            return @splat(-std.math.inf(N));
        }

        /// A vector with the X component equal to -1.
        pub fn left(n: comptime_int) @Vector(n, N) {
            var v = zero(n);
            v[0] = -1.0;
            return v;
        }

        /// A vector with all components set to one.
        pub fn one(n: comptime_int) @Vector(n, N) {
            return @splat(1.0);
        }

        /// A vector with the X component equal to 1.
        pub fn right(n: comptime_int) @Vector(n, N) {
            var v = zero(n);
            v[0] = 1.0;
            return v;
        }

        /// A vector with the Y component equal to 1.
        pub fn up(n: comptime_int) @Vector(n, N) {
            var v = zero(n);
            v[1] = 1.0;
            return v;
        }

        /// A vector with all components equal to zero.
        pub fn zero(n: comptime_int) @Vector(n, N) {
            return @splat(0.0);
        }

        // --- PROPERTIES ---

        /// Returns the length of `v`. If you're comparing lengths, you
        /// should use `sqrMagnitude` instead to save on performance.
        pub fn magnitude(n: comptime_int, v: @Vector(n, N)) N {
            return @sqrt(sqrMagnitude(n, v));
        }

        /// Normalizes `v`. If the length of `v` is zero, this returns the zero vector.
        pub fn normalize(n: comptime_int, v: @Vector(n, N)) @Vector(n, N) {
            const len = magnitude(n, v);
            if (len == 0.0) return @splat(0.0);

            const v_len: @Vector(n, N) = @splat(len);
            return v / v_len;
        }

        /// Returns the square length of `v`.
        pub fn sqrMagnitude(n: comptime_int, v: @Vector(n, N)) N {
            return dot(n, v, v);
        }

        // --- METHODS ---

        /// Returns the angle in radians between `a` and `b`.
        pub fn angle(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N)) N {
            return std.math.acos(
                dot(n, normalize(n, a), normalize(n, b)),
            );
        }

        /// Clamps the length of `v` to `max_len`.
        pub fn clampMagnitude(n: comptime_int, v: @Vector(n, N), max_len: N) @Vector(n, N) {
            const len = magnitude(n, v);
            if (len < max_len) return v;

            const len_v: @Vector(n, N) = @splat(len);
            const max_v: @Vector(n, N) = @splat(max_len);
            return v / len_v * max_v;
        }

        /// Returns the cross product between `a` and `b`.
        pub fn cross(a: @Vector(3, N), b: @Vector(3, N)) @Vector(3, N) {
            const ax, const ay, const az = a;
            const bx, const by, const bz = b;
            const cx = ay * bz - az * by;
            const cy = az * bx - ax * bz;
            const cz = ax * by - ay * bx;
            return .{ cx, cy, cz };
        }

        /// Returns the distance between `a` and `b`.
        pub fn distance(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N)) N {
            return magnitude(n, a - b);
        }

        /// Returns the dot product between `a` and `b`.
        pub fn dot(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N)) N {
            return @reduce(.Add, a * b);
        }

        /// Returns true if `a` and `b` are exactly equal.
        pub fn eql(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N)) bool {
            return @reduce(.And, a == b);
        }

        /// Extends `v` to length `n + 1` by append `scalar` to it.
        pub fn extend(n: comptime_int, v: @Vector(n, N), scalar: N) @Vector(n + 1, N) {
            var w: @Vector(n + 1, N) = undefined;
            for (0..n) |i| w[i] = v[i];
            w[n] = scalar;
            return w;
        }

        /// Linearly interpolates between `a` and `b` by factor `t`.
        pub fn lerp(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N), t: N) @Vector(n, N) {
            return lerpUnclamped(n, a, b, std.math.clamp(t, 0.0, 1.0));
        }

        /// Linearly interpolates between `a` and `b` by factor `t`. Allows over- and under-shooting.
        pub fn lerpUnclamped(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N), t: N) @Vector(n, N) {
            const m = b - a;
            const t_v: @Vector(n, N) = @splat(t);
            return m * t_v + a;
        }

        /// Moves `current` towards `target` by at most `max_delta` units. This will not overshoot.
        pub fn moveTowards(n: comptime_int, current: @Vector(n, N), target: @Vector(n, N), max_delta: N) @Vector(n, N) {
            const d = target - current;
            const dist = sqrMagnitude(n, d);
            if (dist < max_delta * max_delta) return target;

            const dist_v: @Vector(n, N) = @splat(@sqrt(dist));
            const max_v: @Vector(n, N) = @splat(max_delta);
            return current + d / dist_v * max_v;
        }

        /// Returns the 2D vector perpendicular to `v`. This is equivalent to rotating it 90 degress counter-clockwise.
        pub fn perpendicular(v: @Vector(2, N)) @Vector(2, N) {
            return .{ -v[1], v[0] };
        }

        /// Projects `v` onto `normal`.
        pub fn project(n: comptime_int, v: @Vector(n, N), normal: @Vector(n, N)) @Vector(n, N) {
            const d: @Vector(n, N) = @splat(dot(n, v, normal));
            return normal * d;
        }

        /// Reflects `direction` across `normal`.
        pub fn reflect(n: comptime_int, direction: @Vector(n, N), normal: @Vector(n, N)) @Vector(n, N) {
            const d: @Vector(n, N) = @splat(dot(n, direction, normal) * 2.0);
            return direction - d * normal;
        }

        /// Rotates `current` towards `target`, by at most `max_delta_radians`. This will not overshoot.
        pub fn rotateToward(n: comptime_int, current: @Vector(n, N), target: @Vector(n, N), max_delta_radians: N) @Vector(n, N) {
            const current_norm = normalize(n, current);
            const target_norm = normalize(n, target);
            const dot_product = dot(n, current_norm, target_norm);

            // Bail out if vectors are aligned
            if (dot_product > 1.0 - epsilon) return target_norm;

            // If angle is less than max delta, return target
            const a = std.math.acos(dot_product);
            if (a <= max_delta_radians) return target;

            // Calculate the rejection of target from current
            const rejection = target_norm - current_norm * @as(@Vector(n, N), @splat(dot_product));
            const rejection_len = magnitude(n, rejection);

            // Handle case where vectors are parallel or anti-parallel
            var perp: @Vector(n, N) = undefined;
            if (rejection_len < epsilon) {
                // If vectors are nearly parallel, we're already at the target
                if (dot_product > 0) return target_norm;

                // If vectors are nearly anti-parallel, choose an arbitrary perpendicular direction
                // Find index of largest component
                var max_idx: usize = 0;
                var max_val: N = @abs(current_norm[0]);
                inline for (0..n) |i| {
                    if (@abs(current_norm[i]) > max_val) {
                        max_idx = i;
                        max_val = @abs(current_norm[i]);
                    }
                }

                // Create a vector perpendicular to current_norm
                perp = @splat(0.0);
                perp[max_idx] = -current_norm[(max_idx + 1) % n];
                perp[(max_idx + 1) % n] = current_norm[max_idx];
                perp = normalize(n, perp);
            } else {
                perp = normalize(n, rejection);
            }

            // Rotate in the plane defined by current_norm and perp_dir
            const cos_delta = @cos(max_delta_radians);
            const sin_delta = @sin(max_delta_radians);
            return current_norm * @as(@Vector(n, N), @splat(cos_delta)) +
                perp * @as(@Vector(n, N), @splat(sin_delta));
        }

        /// Scales `v` by `scalar`.
        pub fn scale(n: comptime_int, v: @Vector(n, N), scalar: N) @Vector(n, N) {
            const s: @Vector(n, N) = @splat(scalar);
            return v * s;
        }

        /// Spherically interpolates between `a` and `b` by factor `t`.
        pub fn slerp(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N), t: N) @Vector(n, N) {
            return slerpUnclamped(n, a, b, std.math.clamp(t, 0.0, 1.0));
        }

        /// Spherically interpolates between `a` and `b` by factor `t`. Allows over- and under-shooting.
        pub fn slerpUnclamped(n: comptime_int, a: @Vector(n, N), b: @Vector(n, N), t: N) @Vector(n, N) {
            // Get lengths of input vectors
            const len_a = magnitude(n, a);
            const len_b = magnitude(n, b);

            // Handle zero length cases
            if (len_a < epsilon and len_b < epsilon) return @splat(0.0);
            if (len_a < epsilon) return b * @as(@Vector(n, N), @splat(t / len_b));
            if (len_b < epsilon) return a * @as(@Vector(n, N), @splat((1.0 - t) / len_a));

            // Normalize inputs
            const a_norm = a / @as(@Vector(n, f32), @splat(len_a));
            const b_norm = b / @as(@Vector(n, f32), @splat(len_b));
            const dot_product = dot(n, a_norm, b_norm);
            const target_len = len_a * (1 - t) + len_b * t;

            // Calculate the angle between the vectors
            const theta = std.math.acos(dot_product);
            const sin_theta = @sin(theta);
            if (sin_theta < epsilon) return lerp(n, a, b, t); // Linearly interpolate very small angles

            // Calculate SLERP coefficients
            const sin_theta_t = @sin(theta * t);
            const sin_theta_1_minus_t = @sin(theta * (1 - t));

            // Apply SLERP formula
            const s0 = sin_theta_1_minus_t / sin_theta;
            const s1 = sin_theta_t / sin_theta;

            // Calculate the interpolated vector and scale to target length
            const result = (a_norm * @as(@Vector(n, f32), @splat(s0)) +
                b_norm * @as(@Vector(n, f32), @splat(s1))) *
                @as(@Vector(n, f32), @splat(target_len));

            return result;
        }
    };
}
