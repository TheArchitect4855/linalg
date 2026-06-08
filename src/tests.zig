const std = @import("std");
const root = @import("root.zig").With(f32);

test "sanity" {
    try std.testing.expectEqual(2, 1 + 1);
}

test "matrix look at" {
    const m = root.Mat4.lookAt(@splat(0), .{ 0, 0, 1.0 }, .{ 0, 1, 0 });
    const expected = root.Mat4.initRows(
        .{ -1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -1, 0 },
        .{ 0, 0, 0, 1 },
    );
    try std.testing.expectEqualDeep(expected, m);
}

test "matrix orthographic" {
    const m = root.Mat4.orthographic(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    const expected = root.Mat4.initRows(
        .{ 1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -1, 0 },
        .{ 0, 0, 0, 1 },
    );
    try std.testing.expectEqualDeep(expected, m);
}

test "matrix perspective" {
    const m = root.Mat4.perspective(1.0, std.math.pi * 0.5, 1.0, 2.0, -1.0, 1.0);
    const expected = root.Mat4.initRows(
        .{ 1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -3, -4 },
        .{ 0, 0, -1, 0 },
    );

    try std.testing.expectEqualDeep(expected, m);
}

test "matrix transform" {
    const m = root.Mat4.transform(
        .{ 0, 0, 1 },
        root.Quat{ .w = 0, .x = 0, .y = 1, .z = 0 },
        @splat(1),
    );
    const expected = root.Mat4.initRows(
        .{ -1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -1, 1 },
        .{ 0, 0, 0, 1 },
    );
    try std.testing.expectEqualDeep(expected, m);
}

test "matrix determinant" {
    const m = root.Mat4.initRows(
        .{ 0, 1, 2, 3 },
        .{ 4, 5, 6, 7 },
        .{ 8, 9, 10, 11 },
        .{ 12, 13, 14, 15 },
    );
    const det = m.determinant();
    try std.testing.expectEqual(0, det);
}

test "matrix inverse" {
    const m = root.Mat4.initRows(
        .{ 2, 0, 0, 3 },
        .{ 0, 2, 0, 4 },
        .{ 0, 0, 2, 5 },
        .{ 0, 0, 0, 1 },
    );
    const m_inv = m.inverse();
    const i = m.mul(m_inv);
    try std.testing.expectEqualDeep(root.Mat4.identity, i);
}

test "matrix transpose" {
    const m = root.Mat4.initRows(
        .{ 0, 1, 2, 3 },
        .{ 4, 5, 6, 7 },
        .{ 8, 9, 10, 11 },
        .{ 12, 13, 14, 15 },
    ).transpose();
    const expected = root.Mat4{ .m = .{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 } };
    try std.testing.expectEqualDeep(expected, m);
}

test "matrix multiply direction" {
    // Rotate vector 180 degrees around Y
    // Translation & scale should have no effect
    const m = root.Mat4.transform(@splat(1.0), .{ .w = 0, .x = 0, .y = 1, .z = 0 }, @splat(1.0));
    const dir = m.multiplyDirection(root.Vec3{ 1, 0, 0 });
    const expected = root.Vec3{ -1, 0, 0 };
    try std.testing.expectEqualDeep(expected, dir);
}

test "matrix multiply point" {
    // Rotate vector 180 degrees around Y and translate
    const m = root.Mat4.transform(@splat(1.0), .{ .w = 0, .x = 0, .y = 1, .z = 0 }, @splat(1.0));
    const point = m.multiplyPoint(root.Vec3{ 1, 0, 0 });
    const expected = root.Vec3{ 0, 1, 1 };
    try std.testing.expectEqualDeep(expected, point);
}

test "quaternion angle axis" {
    // 180 degrees around X axis
    const q = root.Quat.angleAxis(std.math.pi, root.Vec3{ 1, 0, 0 });
    const expected = root.Quat{ .w = 0, .x = 1, .y = 0, .z = 0 };
    try std.testing.expect(expected.eqlApprox(q, 1e-6));
}

test "quaternion euler" {
    // 180 degrees around Y axis
    const q = root.Quat.euler(0, std.math.pi, 0);
    const expected = root.Quat{ .w = 0, .x = 0, .y = 1, .z = 0 };
    try std.testing.expect(expected.eqlApprox(q, 1e-6));
}

test "quaternion look rotation" {
    const q = root.Quat.lookRotation(.{ 0, 0, 1 }, .{ 0, 1, 0 });
    const expected = root.Quat.identity;
    try std.testing.expectEqualDeep(expected, q);
}

test "quaternion angle" {
    const a = root.Quat.euler(0, 0, 0);
    const b = root.Quat.euler(std.math.pi, 0, 0);
    const delta = a.angle(b);
    try std.testing.expectApproxEqRel(std.math.pi, delta, 1e-6);
}

test "quaternion conjugate" {
    const q = root.Quat{ .w = 1, .x = 2, .y = 3, .z = 4 };
    const r = q.conjugate();
    const expected = root.Quat{ .w = 1, .x = -2, .y = -3, .z = -4 };
    try std.testing.expectEqualDeep(expected, r);
}

test "quaternion inverse" {
    const q = root.Quat{ .w = 0, .x = 1, .y = 2, .z = 3 };
    const q_inv = q.inverse();
    const i = q.mul(q_inv);
    try std.testing.expectEqualDeep(root.Quat.identity, i);
}

test "quaternion lerp" {
    const a = root.Quat.identity;
    const b = root.Quat.euler(std.math.pi, 0, 0);
    const c = a.lerp(b, 0.5);
    const expected = (root.Quat{ .w = 0.5, .x = 0.5, .y = 0.0, .z = 0.0 }).normalized();
    try std.testing.expectEqualDeep(expected, c);
}

test "quaternion rotate towards" {
    const a = root.Quat.identity;
    const b = root.Quat{ .w = 0, .x = 1, .y = 0, .z = 0 };
    const c = a.rotateTowards(b, std.math.pi * 0.5);
    const expected = root.Quat.euler(std.math.pi * 0.5, 0, 0);
    try std.testing.expect(expected.eqlApprox(c, 1e-6));
}

test "quaternion slerp" {
    const a = root.Quat.identity;
    const b = root.Quat{ .w = 0, .x = 1, .y = 0, .z = 0 };
    const c = a.slerp(b, 0.5);
    const expected = root.Quat.euler(std.math.pi * 0.5, 0, 0);
    try std.testing.expect(expected.eqlApprox(c, 1e-6));
}

test "vector angle" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const angle = root.angle(a, b);
    try std.testing.expectApproxEqRel(std.math.pi * 0.5, angle, 1e-6);
}

test "vector clamp magnitude" {
    const v = root.Vec3{ 1, 2, 3 };
    const v_clamped = root.clampMagnitude(v, 1);
    const expected = root.normalize(v);
    try std.testing.expectEqualDeep(expected, v_clamped);
}

test "vector distance" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ -1, 0, 0 };
    const distance = root.distance(a, b);
    try std.testing.expectEqual(2, distance);
}

test "vector extend" {
    const v2 = root.Vec2{ 1, 2 };
    const v3 = root.extend(2, v2, 3);
    const expected = root.Vec3{ 1, 2, 3 };
    try std.testing.expectEqualDeep(expected, v3);
}

test "vector lerp" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const c = root.lerp(a, b, 0.5);
    const expected = root.Vec3{ 0.5, 0.5, 0 };
    try std.testing.expectEqualDeep(expected, c);
}

test "vector move towards" {
    const a = root.Vec3{ 0, 0, 0 };
    const b = root.Vec3{ 0, 0, 1 };
    const c = root.moveTowards(a, b, 0.5);
    const expected = root.Vec3{ 0, 0, 0.5 };
    try std.testing.expectEqualDeep(expected, c);
}

test "vector perpendicular" {
    const v = root.Vec2{ 0, 1 };
    const p = root.perpendicular(v);
    const expected = root.Vec2{ -1, 0 };
    try std.testing.expectEqualDeep(expected, p);
}

test "vector project" {
    const v = root.Vec3{ 5, 5, 0 };
    const n = root.Vec3{ 0, 1, 0 };
    const p = root.project(v, n);
    const expected = root.Vec3{ 0, 5, 0 };
    try std.testing.expectEqualDeep(expected, p);
}

test "vector reflect" {
    const v = root.normalize(root.Vec3{ 1, -1, 0 });
    const n = root.Vec3{ 0, 1, 0 };
    const r = root.reflect(v, n);
    const expected = root.normalize(root.Vec3{ 1, 1, 0 });
    try std.testing.expectEqualDeep(expected, r);
}

test "vector rotate toward" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const c = root.rotateToward(a, b, std.math.pi * 0.25);
    const expected = root.normalize(root.Vec3{ 0.5, 0.5, 0 });
    try std.testing.expectEqualDeep(expected, c);
}

test "vector scale" {
    const v: root.Vec3 = @splat(1.0);
    const scaled = root.scale(v, 5);
    const expected: root.Vec3 = @splat(5.0);
    try std.testing.expectEqualDeep(expected, scaled);
}

test "vector slerp" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const c = root.slerp(a, b, 0.5);
    const expected = root.normalize(root.Vec3{ 0.5, 0.5, 0 });
    try std.testing.expectEqualDeep(expected, c);
}
