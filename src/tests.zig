const std = @import("std");
const root = @import("root.zig").LinAlg(f32);

test "sanity" {
    std.debug.assert(1 + 1 == 2);
}

test "matrix look at" {
    const m = root.Mat4.lookAt(root.vec.zero(3), root.vec.forward(3), root.vec.up(3));
    const expected = root.Mat4.initRows(
        .{ -1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -1, 0 },
        .{ 0, 0, 0, 1 },
    );
    std.debug.assert(m.eql(expected));
}

test "matrix orthographic" {
    const m = root.Mat4.orthographic(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    const expected = root.Mat4.initRows(
        .{ 1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -1, 0 },
        .{ 0, 0, 0, 1 },
    );
    std.debug.assert(m.eql(expected));
}

test "matrix perspective" {
    const m = root.Mat4.perspective(1.0, std.math.pi * 0.5, -1.0, 1.0);
    const expected = root.Mat4.initRows(
        .{ 1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, 0, 1 },
        .{ 0, 0, -1, 0 },
    );
    std.debug.assert(m.eql(expected));
}

test "matrix transform" {
    const m = root.Mat4.transform(
        root.vec.forward(3),
        root.Quat{ .w = 0, .x = 0, .y = 1, .z = 0 },
        root.vec.one(3),
    );
    const expected = root.Mat4.initRows(
        .{ -1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, -1, 1 },
        .{ 0, 0, 0, 1 },
    );
    std.debug.assert(m.eql(expected));
}

test "matrix determinant" {
    const m = root.Mat4.initRows(
        .{ 0, 1, 2, 3 },
        .{ 4, 5, 6, 7 },
        .{ 8, 9, 10, 11 },
        .{ 12, 13, 14, 15 },
    );
    const det = m.determinant();
    std.debug.assert(det == 0);
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
    std.debug.assert(i.eql(.identity));
}

test "matrix transpose" {
    const m = root.Mat4.initRows(
        .{ 0, 1, 2, 3 },
        .{ 4, 5, 6, 7 },
        .{ 8, 9, 10, 11 },
        .{ 12, 13, 14, 15 },
    );
    const expected = root.Mat4{ .m = .{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 } };
    std.debug.assert(m.transpose().eql(expected));
}

test "matrix multiply direction" {
    // Rotate vector 180 degrees around Y
    // Translation & scale should have no effect
    const m = root.Mat4.transform(root.vec.one(3), .{ .w = 0, .x = 0, .y = 1, .z = 0 }, root.vec.one(3));
    const dir = m.multiplyDirection(root.Vec3{ 1, 0, 0 });
    const expected = root.Vec3{ -1, 0, 0 };
    std.debug.assert(root.vec.eql(3, dir, expected));
}

test "matrix multiply point" {
    // Rotate vector 180 degrees around Y and translate
    const m = root.Mat4.transform(root.vec.one(3), .{ .w = 0, .x = 0, .y = 1, .z = 0 }, root.vec.one(3));
    const point = m.multiplyPoint(root.Vec3{ 1, 0, 0 });
    const expected = root.Vec3{ 0, 1, 1 };
    std.debug.assert(root.vec.eql(3, point, expected));
}

test "quaternion angle axis" {
    // 180 degrees around X axis
    const q = root.Quat.angleAxis(std.math.pi, root.Vec3{ 1, 0, 0 });
    const expected = root.Quat{ .w = 0, .x = 1, .y = 0, .z = 0 };
    return std.debug.assert(q.eqlApprox(expected));
}

test "quaternion euler" {
    // 180 degrees around Y axis
    const q = root.Quat.euler(0, std.math.pi, 0);
    const expected = root.Quat{ .w = 0, .x = 0, .y = 1, .z = 0 };
    return std.debug.assert(q.eqlApprox(expected));
}

test "quaternion look rotation" {
    const q = root.Quat.lookRotation(root.vec.forward(3), root.vec.up(3));
    const expected = root.Quat.identity;
    std.debug.assert(q.eql(expected));
}

test "quaternion angle" {
    const a = root.Quat.euler(0, 0, 0);
    const b = root.Quat.euler(std.math.pi, 0, 0);
    const delta = a.angle(b);
    std.debug.assert(std.math.approxEqRel(f32, delta, std.math.pi, 1e-6));
}

test "quaternion inverse" {
    const q = root.Quat{ .w = 0, .x = 1, .y = 2, .z = 3 };
    const q_inv = q.inverse();
    const i = q.mul(q_inv);
    std.debug.assert(i.eql(root.Quat.identity));
}

test "quaternion lerp" {
    const a = root.Quat.identity;
    const b = root.Quat.euler(std.math.pi, 0, 0);
    const c = a.lerp(b, 0.5);
    const expected = (root.Quat{ .w = 0.5, .x = 0.5, .y = 0.0, .z = 0.0 }).normalized();
    std.debug.assert(c.eql(expected));
}

test "quaternion rotate towards" {
    const a = root.Quat.identity;
    const b = root.Quat{ .w = 0, .x = 1, .y = 0, .z = 0 };
    const c = a.rotateTowards(b, std.math.pi * 0.5);
    const expected = root.Quat.euler(std.math.pi * 0.5, 0, 0);
    std.debug.assert(c.eqlApprox(expected));
}

test "quaternion slerp" {
    const a = root.Quat.identity;
    const b = root.Quat{ .w = 0, .x = 1, .y = 0, .z = 0 };
    const c = a.slerp(b, 0.5);
    const expected = root.Quat.euler(std.math.pi * 0.5, 0, 0);
    std.debug.assert(c.eql(expected));
}

test "vector angle" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const angle = root.vec.angle(3, a, b);
    std.debug.assert(std.math.approxEqRel(f32, angle, std.math.pi * 0.5, 1e-6));
}

test "vector clamp magnitude" {
    const v = root.Vec3{ 1, 2, 3 };
    const v_clamped = root.vec.clampMagnitude(3, v, 1);
    const expected = root.vec.normalize(3, v);
    std.debug.assert(root.vec.eql(3, v_clamped, expected));
}

test "vector distance" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ -1, 0, 0 };
    const distance = root.vec.distance(3, a, b);
    std.debug.assert(distance == 2);
}

test "vector extend" {
    const v2 = root.Vec2{ 1, 2 };
    const v3 = root.vec.extend(2, v2, 3);
    const expected = root.Vec3{ 1, 2, 3 };
    std.debug.assert(root.vec.eql(3, v3, expected));
}

test "vector lerp" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const c = root.vec.lerp(3, a, b, 0.5);
    const expected = root.Vec3{ 0.5, 0.5, 0 };
    std.debug.assert(root.vec.eql(3, c, expected));
}

test "vector move towards" {
    const a = root.Vec3{ 0, 0, 0 };
    const b = root.Vec3{ 0, 0, 1 };
    const c = root.vec.moveTowards(3, a, b, 0.5);
    const expected = root.Vec3{ 0, 0, 0.5 };
    std.debug.assert(root.vec.eql(3, c, expected));
}

test "vector perpendicular" {
    const v = root.Vec2{ 0, 1 };
    const p = root.vec.perpendicular(v);
    const expected = root.Vec2{ -1, 0 };
    std.debug.assert(root.vec.eql(2, p, expected));
}

test "vector project" {
    const v = root.Vec3{ 5, 5, 0 };
    const n = root.Vec3{ 0, 1, 0 };
    const p = root.vec.project(3, v, n);
    const expected = root.Vec3{ 0, 5, 0 };
    std.debug.assert(root.vec.eql(3, p, expected));
}

test "vector reflect" {
    const v = root.vec.normalize(3, root.Vec3{ 1, -1, 0 });
    const n = root.Vec3{ 0, 1, 0 };
    const r = root.vec.reflect(3, v, n);
    const expected = root.vec.normalize(3, root.Vec3{ 1, 1, 0 });
    std.debug.assert(root.vec.eql(3, r, expected));
}

test "vector rotate toward" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const c = root.vec.rotateToward(3, a, b, std.math.pi * 0.25);
    const expected = root.vec.normalize(3, root.Vec3{ 0.5, 0.5, 0 });
    std.debug.assert(root.vec.eql(3, c, expected));
}

test "vector scale" {
    const v = root.vec.one(3);
    const scaled = root.vec.scale(3, v, 5);
    const expected: root.Vec3 = @splat(5.0);
    std.debug.assert(root.vec.eql(3, scaled, expected));
}

test "vector slerp" {
    const a = root.Vec3{ 1, 0, 0 };
    const b = root.Vec3{ 0, 1, 0 };
    const c = root.vec.slerp(3, a, b, 0.5);
    const expected = root.vec.normalize(3, root.Vec3{ 0.5, 0.5, 0 });
    std.debug.assert(root.vec.eql(3, c, expected));
}
