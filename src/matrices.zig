const std = @import("std");
const q = @import("quaternion.zig");

/// A column-major 3x3 matrix type.
pub fn Mat3(N: type) type {
    return extern struct {
        const Self = @This();

        pub const identity = Self{ .m = .{ 1, 0, 0, 0, 1, 0, 0, 0, 1 } };
        pub const zero = Self{ .m = .{ 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

        m: [9]N,

        // --- CONSTRUCTORS ---

        /// Utility to initialize a matrix row-by-row. This essentially tranposes the input
        /// so that the matrix is stored in column-major order.
        pub fn initRows(x: [4]f32, y: [4]f32, z: [4]f32) Self {
            return .{
                .m = .{
                    x[0], y[0], z[0], //
                    x[1], y[1], z[1], //
                    x[2], y[2], z[2], //
                    x[3], y[3], z[3], //
                },
            };
        }

        // --- PROPERTIES ---

        pub fn inverse(self: Self) Self {
            const a = self.m[0];
            const b = self.m[1];
            const c = self.m[2];
            const d = self.m[3];
            const e = self.m[4];
            const f = self.m[5];
            const g = self.m[6];
            const h = self.m[7];
            const i = self.m[8];

            const inv_det = 1.0 / (a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g));
            var result: [9]f32 = undefined;
            result[0] = (e * i - f * h) * inv_det;
            result[1] = -(b * i - c * h) * inv_det;
            result[2] = (b * f - c * e) * inv_det;
            result[3] = -(d * i - f * g) * inv_det;
            result[4] = (a * i - c * g) * inv_det;
            result[5] = -(a * f - c * d) * inv_det;
            result[6] = (d * h - e * g) * inv_det;
            result[7] = -(a * h - b * g) * inv_det;
            result[8] = (a * e - b * d) * inv_det;
            return .{ .m = result };
        }

        /// Returns the transpose of this matrix.
        pub fn transpose(self: Self) Self {
            var result: Self = undefined;
            inline for (0..3) |row| {
                inline for (0..3) |col| {
                    result.m[col * 3 + row] = self.m[row * 3 + col];
                }
            }

            return result;
        }

        // --- METHODS ---

        /// Returns true if `self` is exactly equal to `other`.
        pub fn eql(self: Self, other: Self) bool {
            return std.mem.eql(N, &self.m, &other.m);
        }

        /// Format function for printing. One can use the `{d}` format specifier in the same was as printing a number.
        pub fn format(self: Self, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
            _ = options;
            try writer.print("[[ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]\n", .{ self.m[0], self.m[3], self.m[6] });
            try writer.print(" [ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]\n", .{ self.m[1], self.m[4], self.m[7] });
            try writer.print(" [ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]\n", .{ self.m[2], self.m[5], self.m[8] });
            try writer.print(" [ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]]", .{ self.m[3], self.m[6], self.m[9] });
        }

        /// Returns `self` * `other`.
        pub fn mul(self: Self, other: Self) Self {
            var result: [9]N = undefined;
            inline for (0..3) |col| {
                inline for (0..3) |row| {
                    var sum: N = 0.0;
                    inline for (0..3) |i| sum += self.m[i * 3 + row] * other.m[col * 3 + i];
                    result[col * 3 + row] = sum;
                }
            }

            return .{ .m = result };
        }

        /// Multiplies all elements in `self` with `scalar`.
        pub fn mulScalar(self: Self, scalar: N) Self {
            var result: [9]N = undefined;
            inline for (0..9) |i| result[i] = self.m[i] * scalar;
            return .{ .m = result };
        }

        /// Multiplies a point by this matrix.
        pub fn multiplyPoint(self: Self, point: @Vector(3, N)) @Vector(3, N) {
            const m = self.m;
            const x = m[0] * point[0] + m[3] * point[1] + m[6] * point[2];
            const y = m[1] * point[0] + m[4] * point[1] + m[7] * point[2];
            const z = m[2] * point[0] + m[5] * point[1] + m[8] * point[2];
            return .{ x, y, z };
        }
    };
}

/// A columm-major 4x4 matrix type.
pub fn Mat4(N: type) type {
    const Q = q.Quat(N);

    const v = @import("root.zig").With(N);
    return extern struct {
        const Self = @This();

        pub const identity = Self{ .m = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } };
        pub const zero = Self{ .m = .{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

        m: [16]N,

        // --- CONSTRUCTORS ---

        /// Utility to initialize a matrix row-by-row. This essentially tranposes the input
        /// so that the matrix is stored in column-major order.
        pub fn initRows(x: [4]f32, y: [4]f32, z: [4]f32, w: [4]f32) Self {
            return .{
                .m = .{
                    x[0], y[0], z[0], w[0], //
                    x[1], y[1], z[1], w[1], //
                    x[2], y[2], z[2], w[2], //
                    x[3], y[3], z[3], w[3], //
                },
            };
        }

        /// Creates a "look at" matrix at position `from` looking at `to`.
        pub fn lookAt(from: @Vector(3, N), to: @Vector(3, N), up: @Vector(3, N)) Self {
            const forward = v.normalize(from - to);
            const right = v.cross(up, forward);
            const new_up = v.cross(forward, right);

            // Create rotation matrix (first 3x3 part)
            var result = identity;
            result.m[0] = right[0];
            result.m[1] = right[1];
            result.m[2] = right[2];

            result.m[4] = new_up[0];
            result.m[5] = new_up[1];
            result.m[6] = new_up[2];

            result.m[8] = forward[0];
            result.m[9] = forward[1];
            result.m[10] = forward[2];

            // Apply translation
            result.m[12] = -v.dot(right, from);
            result.m[13] = -v.dot(new_up, from);
            result.m[14] = -v.dot(forward, from);

            return result;
        }

        /// Creates an orthographic projection matrix.
        pub fn orthographic(left: N, right: N, bottom: N, top: N, near: N, far: N) Self {
            var result = identity;

            // Scale
            result.m[0] = 2.0 / (right - left);
            result.m[5] = 2.0 / (top - bottom);
            result.m[10] = -2.0 / (far - near);

            // Translation
            result.m[12] = -(right + left) / (right - left);
            result.m[13] = -(top + bottom) / (top - bottom);
            result.m[14] = -(far + near) / (far - near);

            return result;
        }

        /// Creates a perspective projection matrix. `fov` is in radians.
        pub fn perspective(aspect: N, fov: N, near: N, far: N, clip_min: N, clip_max: N) Self {
            const tan_half_fov = std.math.tan(fov * 0.5);

            var result = zero;
            result.m[0] = 1.0 / tan_half_fov; // xx
            result.m[5] = aspect / tan_half_fov; // yy
            result.m[10] = -(far * clip_max - near * clip_min) / (far - near); // zz
            result.m[14] = (far * near * clip_min - near * far * clip_max) / (far - near); // zw
            result.m[11] = -1; // wz
            return result;
        }

        /// Creates a 3D transformation matrix.
        pub fn transform(translation: @Vector(3, N), rotation: Q, scale: @Vector(3, N)) Self {
            const rw = rotation.w;
            const rx = rotation.x;
            const ry = rotation.y;
            const rz = rotation.z;

            var result = [1]f32{0.0} ** 16;
            result[0] = (1.0 - 2.0 * (ry * ry + rz * rz)) * scale[0];
            result[1] = (2.0 * (rx * ry + rw * rz)) * scale[0];
            result[2] = (2.0 * (rx * rz - rw * ry)) * scale[0];

            result[4] = (2.0 * (rx * ry - rw * rz)) * scale[1];
            result[5] = (1.0 - 2.0 * (rx * rx + rz * rz)) * scale[1];
            result[6] = (2.0 * (ry * rz + rw * rx)) * scale[1];

            result[8] = (2.0 * (rx * rz + rw * ry)) * scale[2];
            result[9] = (2.0 * (ry * rz - rw * rx)) * scale[2];
            result[10] = (1.0 - 2.0 * (rx * rx + ry * ry)) * scale[2];

            result[12] = translation[0];
            result[13] = translation[1];
            result[14] = translation[2];
            result[15] = 1;
            return .{ .m = result };
        }

        // --- PROPERTIES ---

        /// The determinant of this matrix.
        pub fn determinant(self: Self) N {
            const m = self.m;

            // Calculate the cofactors for the first row
            const c00 = m[5] * (m[10] * m[15] - m[11] * m[14]) -
                m[6] * (m[9] * m[15] - m[11] * m[13]) +
                m[7] * (m[9] * m[14] - m[10] * m[13]);

            const c01 = m[4] * (m[10] * m[15] - m[11] * m[14]) -
                m[6] * (m[8] * m[15] - m[11] * m[12]) +
                m[7] * (m[8] * m[14] - m[10] * m[12]);

            const c02 = m[4] * (m[9] * m[15] - m[11] * m[13]) -
                m[5] * (m[8] * m[15] - m[11] * m[12]) +
                m[7] * (m[8] * m[13] - m[9] * m[12]);

            const c03 = m[4] * (m[9] * m[14] - m[10] * m[13]) -
                m[5] * (m[8] * m[14] - m[10] * m[12]) +
                m[6] * (m[8] * m[13] - m[9] * m[12]);

            // Calculate the determinant using the cofactors
            return m[0] * c00 - m[1] * c01 + m[2] * c02 - m[3] * c03;
        }

        /// Returns the inverse of this matrix. If there is no inverse,
        /// the matrix will be all `NaN`s.
        pub fn inverse(self: Self) Self {
            const m = self.m;

            // Calculate cofactors and determinant
            const c00 = m[5] * (m[10] * m[15] - m[11] * m[14]) -
                m[6] * (m[9] * m[15] - m[11] * m[13]) +
                m[7] * (m[9] * m[14] - m[10] * m[13]);

            const c01 = m[4] * (m[10] * m[15] - m[11] * m[14]) -
                m[6] * (m[8] * m[15] - m[11] * m[12]) +
                m[7] * (m[8] * m[14] - m[10] * m[12]);

            const c02 = m[4] * (m[9] * m[15] - m[11] * m[13]) -
                m[5] * (m[8] * m[15] - m[11] * m[12]) +
                m[7] * (m[8] * m[13] - m[9] * m[12]);

            const c03 = m[4] * (m[9] * m[14] - m[10] * m[13]) -
                m[5] * (m[8] * m[14] - m[10] * m[12]) +
                m[6] * (m[8] * m[13] - m[9] * m[12]);

            const c10 = m[1] * (m[10] * m[15] - m[11] * m[14]) -
                m[2] * (m[9] * m[15] - m[11] * m[13]) +
                m[3] * (m[9] * m[14] - m[10] * m[13]);

            const c11 = m[0] * (m[10] * m[15] - m[11] * m[14]) -
                m[2] * (m[8] * m[15] - m[11] * m[12]) +
                m[3] * (m[8] * m[14] - m[10] * m[12]);

            const c12 = m[0] * (m[9] * m[15] - m[11] * m[13]) -
                m[1] * (m[8] * m[15] - m[11] * m[12]) +
                m[3] * (m[8] * m[13] - m[9] * m[12]);

            const c13 = m[0] * (m[9] * m[14] - m[10] * m[13]) -
                m[1] * (m[8] * m[14] - m[10] * m[12]) +
                m[2] * (m[8] * m[13] - m[9] * m[12]);

            const c20 = m[1] * (m[6] * m[15] - m[7] * m[14]) -
                m[2] * (m[5] * m[15] - m[7] * m[13]) +
                m[3] * (m[5] * m[14] - m[6] * m[13]);

            const c21 = m[0] * (m[6] * m[15] - m[7] * m[14]) -
                m[2] * (m[4] * m[15] - m[7] * m[12]) +
                m[3] * (m[4] * m[14] - m[6] * m[12]);

            const c22 = m[0] * (m[5] * m[15] - m[7] * m[13]) -
                m[1] * (m[4] * m[15] - m[7] * m[12]) +
                m[3] * (m[4] * m[13] - m[5] * m[12]);

            const c23 = m[0] * (m[5] * m[14] - m[6] * m[13]) -
                m[1] * (m[4] * m[14] - m[6] * m[12]) +
                m[2] * (m[4] * m[13] - m[5] * m[12]);

            const c30 = m[1] * (m[6] * m[11] - m[7] * m[10]) -
                m[2] * (m[5] * m[11] - m[7] * m[9]) +
                m[3] * (m[5] * m[10] - m[6] * m[9]);

            const c31 = m[0] * (m[6] * m[11] - m[7] * m[10]) -
                m[2] * (m[4] * m[11] - m[7] * m[8]) +
                m[3] * (m[4] * m[10] - m[6] * m[8]);

            const c32 = m[0] * (m[5] * m[11] - m[7] * m[9]) -
                m[1] * (m[4] * m[11] - m[7] * m[8]) +
                m[3] * (m[4] * m[9] - m[5] * m[8]);

            const c33 = m[0] * (m[5] * m[10] - m[6] * m[9]) -
                m[1] * (m[4] * m[10] - m[6] * m[8]) +
                m[2] * (m[4] * m[9] - m[5] * m[8]);

            // Calculate determinant
            const det = m[0] * c00 - m[1] * c01 + m[2] * c02 - m[3] * c03;
            const inv_det = 1.0 / det;
            var result: Self = undefined;
            result.m[0] = c00 * inv_det;
            result.m[1] = -c10 * inv_det;
            result.m[2] = c20 * inv_det;
            result.m[3] = -c30 * inv_det;

            result.m[4] = -c01 * inv_det;
            result.m[5] = c11 * inv_det;
            result.m[6] = -c21 * inv_det;
            result.m[7] = c31 * inv_det;

            result.m[8] = c02 * inv_det;
            result.m[9] = -c12 * inv_det;
            result.m[10] = c22 * inv_det;
            result.m[11] = -c32 * inv_det;

            result.m[12] = -c03 * inv_det;
            result.m[13] = c13 * inv_det;
            result.m[14] = -c23 * inv_det;
            result.m[15] = c33 * inv_det;
            return result;
        }

        /// Returns the transpose of this matrix.
        pub fn transpose(self: Self) Self {
            var result: Self = undefined;
            inline for (0..4) |row| {
                inline for (0..4) |col| {
                    result.m[col * 4 + row] = self.m[row * 4 + col];
                }
            }

            return result;
        }

        // --- METHODS ---

        /// Returns true if `self` is exactly equal to `other`.
        pub fn eql(self: Self, other: Self) bool {
            return std.mem.eql(N, &self.m, &other.m);
        }

        /// Format function for printing. One can use the `{d}` format specifier in the same was as printing a number.
        pub fn format(self: Self, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
            _ = options;
            try writer.print("[[ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]\n", .{ self.m[0], self.m[4], self.m[8], self.m[12] });
            try writer.print(" [ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]\n", .{ self.m[1], self.m[5], self.m[9], self.m[13] });
            try writer.print(" [ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]\n", .{ self.m[2], self.m[6], self.m[10], self.m[14] });
            try writer.print(" [ {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]]", .{ self.m[3], self.m[7], self.m[11], self.m[15] });
        }

        /// Returns `self` * `other`.
        pub fn mul(self: Self, other: Self) Self {
            var result: @Vector(16, f32) = undefined;
            inline for (0..4) |col| {
                inline for (0..4) |row| {
                    var sum: f32 = 0.0;
                    inline for (0..4) |i| sum += self.m[i * 4 + row] * other.m[col * 4 + i];
                    result[col * 4 + row] = sum;
                }
            }

            return .{ .m = result };
        }

        /// Multiplies a direction vector by this matrix, i.e. this only applies rotation.
        pub fn multiplyDirection(self: Self, dir: @Vector(3, N)) @Vector(3, N) {
            const m = self.m;
            return @Vector(3, N){
                m[0] * dir[0] + m[4] * dir[1] + m[8] * dir[2],
                m[1] * dir[0] + m[5] * dir[1] + m[9] * dir[2],
                m[2] * dir[0] + m[6] * dir[1] + m[10] * dir[2],
            };
        }

        /// Multiplies a point by this matrix, i.e. the point is scaled, rotated, and translated.
        pub fn multiplyPoint(self: Self, point: @Vector(3, N)) @Vector(3, N) {
            const m = self.m;
            const x = m[0] * point[0] + m[4] * point[1] + m[8] * point[2] + m[12];
            const y = m[1] * point[0] + m[5] * point[1] + m[9] * point[2] + m[13];
            const z = m[2] * point[0] + m[6] * point[1] + m[10] * point[2] + m[14];
            const w = m[3] * point[0] + m[7] * point[1] + m[11] * point[2] + m[15];
            if (w == 0.0) return .{ x, y, z };

            const inv_w: @Vector(3, N) = @splat(1.0 / w);
            return @Vector(3, N){ x, y, z } * inv_w;
        }
    };
}
