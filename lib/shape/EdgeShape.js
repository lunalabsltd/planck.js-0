/*
 * Copyright (c) 2016-2018 Ali Shakiba http://shakiba.me/planck.js
 * Copyright (c) 2006-2011 Erin Catto  http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

var _DEBUG = typeof DEBUG === 'undefined' ? false : DEBUG;
var _ASSERT = typeof ASSERT === 'undefined' ? false : ASSERT;

module.exports = EdgeShape;

var create = require('../util/create');
var options = require('../util/options');
var Settings = require('../Settings');
var Shape = require('../Shape');
var Math = require('../common/Math');
var Transform = require('../common/Transform');
var Rot = require('../common/Rot');
var Vec2 = require('../common/Vec2');
var AABB = require('../collision/AABB');

var p = new Vec2(), p1 = new Vec2(), p2 = new Vec2(), v1 = new Vec2(), v2 = new Vec2(), d = new Vec2();

EdgeShape._super = Shape;
EdgeShape.prototype = create(EdgeShape._super.prototype);

EdgeShape.TYPE = 'edge';

/**
 * A line segment (edge) shape. These can be connected in chains or loops to
 * other edge shapes. The connectivity information is used to ensure correct
 * contact normals.
 *
 * @param {Number} radius Radius extending around the edge.
 */
function EdgeShape(v1, v2, radius) {
  if (!(this instanceof EdgeShape)) {
    return new EdgeShape(v1, v2);
  }

  EdgeShape._super.call(this);

  this.m_type = EdgeShape.TYPE;
  this.m_radius = Settings.polygonRadius;

  // These are the edge vertices
  this.m_vertex1 = v1 ? Vec2.clone(v1) : Vec2.zero();
  this.m_vertex2 = v2 ? Vec2.clone(v2) : Vec2.zero();

  if (radius > 0.0) {
    this.m_edgeRadius = radius;
    this.m_radius += radius;
    this.m_noAlignRot = true; // This rotation is used to align the edge to y axis,
                              // so v1 is in origin and (v1, v2) is codirectional with y.
  } else {
    this.m_edgeRadius = 0.0;
  }

  // Optional adjacent vertices. These are used for smooth collision.
  // Used by chain shape.
  this.m_vertex0 = Vec2.zero();
  this.m_vertex3 = Vec2.zero();
  this.m_hasVertex0 = false;
  this.m_hasVertex3 = false;
}

EdgeShape.prototype.setNext = function(v3) {
  if (v3) {
    this.m_vertex3.set(v3);
    this.m_hasVertex3 = true;
  } else {
    this.m_vertex3.setZero();
    this.m_hasVertex3 = false;
  }
  return this;
};

EdgeShape.prototype.setPrev = function(v0) {
  if (v0) {
    this.m_vertex0.set(v0);
    this.m_hasVertex0 = true;
  } else {
    this.m_vertex0.setZero();
    this.m_hasVertex0 = false;
  }
  return this;
};

/**
 * Set this as an isolated edge.
 */
EdgeShape.prototype._set = function(v1, v2) {
  this.m_vertex1.set(v1);
  this.m_vertex2.set(v2);
  this.m_hasVertex0 = false;
  this.m_hasVertex3 = false;
  this.m_noAlignRot = true;
  return this;
}

/**
 * @deprecated
 */
EdgeShape.prototype._clone = function() {
  var clone = new EdgeShape();
  clone.m_type = this.m_type;
  clone.m_radius = this.m_radius;
  clone.m_vertex1.set(this.m_vertex1);
  clone.m_vertex2.set(this.m_vertex2);
  clone.m_vertex0.set(this.m_vertex0);
  clone.m_vertex3.set(this.m_vertex3);
  clone.m_hasVertex0 = this.m_hasVertex0;
  clone.m_hasVertex3 = this.m_hasVertex3;
  clone.m_edgeRadius = this.m_edgeRadius;
  return clone;
}

EdgeShape.prototype.getChildCount = function() {
  return 1;
}

/**
 * Test a point for containment in this shape.
 * 
 * @param {Transform} xf The shape world transform.
 * @param {Vec2} p A point in world coordinates.
 * @return {boolean} True if the point is included into the shape, false otherwise.
 */
EdgeShape.prototype.testPoint = function(xf, p) {
  if (this.m_edgeRadius <= 0.0) { // No radius => we never inside.
    return false;
  }
  // Put the point into the edge's frame of reference.
  p = p1.set(p).sub(xf.p).rotT(xf.q);

  // Transform all points so v2 is on y axis, positive half-space, v1 is in (0, 0).
  if (this.m_noAlignRot) {
    this.m_alignRot = this._getAlignRot();
    this.m_alignedV2 = this._alignPoint( (this.m_alignedV2 || new Vec2()).set(this.m_vertex2) );
  }
  p = this._alignPoint(p);
  var v2 = this.m_alignedV2;

  var radius = this.m_edgeRadius;
  if (p.y > v2.y) {
    var dx = p.x - v2.x, dy = p.y - v2.y;
    return dx * dx + dy * dy < radius * radius;
  }
  if (p.y < 0.0) {
    return p.x * p.x + p.y * p.y < radius * radius;
  }
  return -radius < p.x && p.x < radius;
};

/**
 * Creates a rotation to be used to align the edge to y axis.
 *
 * @return {Rot} Rotation used to align points.
 */
EdgeShape.prototype._getAlignRot = function() {
  var rot = this.m_alignRot = this.m_alignRot || new Rot();
  this.m_noAlignRot = false;

  var v1 = this.m_vertex1, v2 = this.m_vertex2;
  var length = d.set(v2).sub(v1).length();

  if (length <= Math.EPSILON) {
    return rot;
  }

  // cos = ( x1 * x2 + y1 * y2 ) / ( length1 * length2 )
  // sin = ( x1 * y2 - x2 * y1 ) / ( length1 * length2 )
  // x1 = d.x, y1 = d.y
  // x2 = 0, y2 = 1
  // length1 = length, length2 = 1
  rot.c = d.y / length;
  rot.s = d.x / length;

  return rot;
};

/**
 * Inline-transforms provided point so it's in coordinate system where current edge is aligned to y axis.
 *
 * @param {Vec2} p Point.
 * @return {Vec2} The transformed point.
 */
EdgeShape.prototype._alignPoint = function(p) {
  return p.sub(this.m_vertex1).rot(this.m_alignRot);
};

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
EdgeShape.prototype.rayCast = function(output, input, xf, childIndex) {
  // NOT_USED(childIndex);

  if (this.m_edgeRadius > 0.0) {
    return this.rayCastWithRadius(output, input, xf);
  }

  v1 = v1.set(this.m_vertex1);
  v2 = v2.set(this.m_vertex2);
  v2 = v2.sub(v1);
  var r = v2;
  p = p.set(r.y, -r.x);
  var normal = p;
  var length = normal.normalize();

  if (length <= Math.EPSILON) { // The edge is a point, can't intersect.
    return false;
  }

  // Put the ray into the edge's frame of reference.
  p1 = p1.set(input.p1).sub(xf.p).rotT(xf.q);
  p2 = p2.set(input.p2).sub(xf.p).rotT(xf.q);
  d = d.set(p2).sub(p1);

  // q = p1 + t * d
  // dot(normal, q - v1) = 0
  // dot(normal, p1 - v1) + t * dot(normal, d) = 0
  var denominator = Vec2.dot(normal, d);

  if (denominator <= Math.EPSILON) { // Ray is parallel to the edge.
    return false;
  }

  p2 = p2.set(v1).sub(p1);
  var numerator = Vec2.dot(normal, p2);

  var t = numerator / denominator;
  if (t < 0.0 || input.maxFraction < t) {
    return false;
  }

  // q = v1 + s * r
  // s = dot(q - v1, r) / dot(r, r)
  var rr = length * length;

  // q = p1 + t * d
  d = d.mul(t);
  p1 = p1.add(d);
  var q = p1;

  q = q.sub(v1);
  var s = Vec2.dot(q, r) / rr;
  if (s < 0.0 || 1.0 < s) {
    return false;
  }

  output.fraction = t;
  if (numerator > 0.0) {
    output.normal = normal.rot(xf.q).neg(); // We are safe to return a reference as it's coppied in c# wrapper.
  } else {
    output.normal = normal.rot(xf.q); // We are safe to return a reference as it's coppied in c# wrapper.
  }
  return true;
};

/**
 * Used internally to do ray casts if current edge has positive radius.
 *
 * @param {RayCastOutput} output The ray-cast results.
 * @param {RayCastInput} input The ray-cast input parameters.
 * @param {Transform} transform The transform to be applied to the shape.
 */
EdgeShape.prototype.rayCastWithRadius = function(output, input, xf) {
  if (this.m_noAlignRot) {
    this.m_alignRot = this._getAlignRot();
    this.m_alignedV2 = this._alignPoint( (this.m_alignedV2 || new Vec2()).set(this.m_vertex2) );
  }

  var radius = this.m_edgeRadius;

  // Put the ray into the edge's frame of reference.
  p1 = this._alignPoint( p1.set(input.p1).sub(xf.p).rotT(xf.q) );
  p2 = this._alignPoint( p2.set(input.p2).sub(xf.p).rotT(xf.q) );
  d = d.set(p2).sub(p1);
  var dx = d.x;

  if (-Math.EPSILON <= dx && dx <= Math.EPSILON) { // The ray runs parallel to edge y axis.
    var px = p1.x; // Intersection point x component.
    if (px <= -radius || radius <= px) { // The ray runs parallel to the edge and fully outside of its radius.
      return false;
    }
    var height = this.m_alignedV2.y;
    var ry = Math.sqrt(radius * radius - px * px);
    var py = height + ry; // Intersection point y component.

    if (p1.y >= py) { // The ray starts above top.
      if (p2.y >= py) { // The ray ends above top, it's fully outside.
        return false;
      }
      output.fraction = (p1.y - py) / d.length();
      output.normal = p.set(px, ry); // Will be coppied in c# wrapper.
      output.normal.mul(1.0 / radius);
    } else if (p1.y <= -ry) { // The ray starts below bottom.
      if (p2.y <= -ry) { // The ray ends below bottom, it's fully outside.
        return false;
      }
      output.fraction = (-p1.y - ry) / d.length();
      output.normal = p.set(px, -ry); // Will be coppied in c# wrapper.
      output.normal.mul(1.0 / radius);
    } else { // The ray starts between top and bottom.
      output.fraction = 0.0;
      output.normal = Vec2.ZERO; // Will be coppied in c# wrapper.
    }
  } else { // The ray is not parallel to edge y axis.
    var p1x = p1.x;
    var t1 = (-radius - p1x) / dx; // Intersection points of the ray line and cylinder containing edge with radius.
    var t2 = (radius - p1x) / dx;

    if ( (t1 <= 0.0 && t2 <= 0.0) || (t1 >= 1.0 && t2 >= 1.0) ) { // Intersections are outside of the ray.
      return false;
    }

    if (t1 > t2) {
      var t = t1;
      t1 = t2;
      t2 = t;
    }

    var dy = d.y;
    var p1y = p1.y;
    var y1 = p1y + t1 * dy;
    if (y1 > height) { // Line intersects cylinder above edge top.
      var a = dx * dx + dy * dy;
      var c = p1x * p1x + (p1y - height) * (p1y - height) - radius * radius;
      var k = p1x * dx + dy * (p1y - height);
      var discr = k * k - a * c;

      if (discr <= Math.EPSILON) { // Line doesn't intersect top circle.
        return false;
      }

      var discrSqrt = Math.sqrt(discr);
      var t3 = (-k - discrSqrt) / a; // Line and top circle intersection closest to start point.

      if (t3 >= 0.0) { 
        if (t3 < 1.0) { // Intersection is inside of the segment.
          output.fraction = t3;
          output.normal = p.set(p1x + t3 * dx, p1y + t3 * dy - height); // Will be coppied in c# wrapper.
          output.normal.mul(1.0 / radius);
        } else { // Intersection is after segment ends.
          return false;
        }
      } else { // The ray is pointing away from the line and top circle first intersection.
        var t4 = (-k + discrSqrt) / a; 
        var y4 = p1y + t4 * dy;

        if (y4 >= height) { // Line and top circle intersection happens on the shape surface.
          if (t4 <= 0.0) { // Start point is outside of semicircle.
            return false;
          }
          // Start point is inside of semicircle.
        } else { // Line intersects bottom semicircle of the top circle.
          var y2 = p1y + t2 * dy; // Line and cylinder second intersection point.
          if (y2 < 0.0) { // Line intersects cylinder below edge bottom, i. e. intersects bottom circle.

            c += height * (2 * p1y - height);
            k += dy * height;
            discr = k * k - a * c;

            if (discr <= 0.0) { // Line doesn't intersect bottom circle or has single intersection point, that should never happen.
              return null;
            }

            discrSqrt = Math.sqrt(discr);
            t4 = (-k + discrSqrt) / a;

            if (t4 <= 0.0) { // The ray is pointing away from bottom circle, no intersections.
              return false;
            }
            // The ray starts inside of the shape.
          } // Line intersects cylinder inside of the shape.
        }
        output.fraction = 0.0;
        output.normal = Vec2.ZERO; // Will be coppied in c# wrapper.
      }
    } else if (y1 < 0.0) { // Line intersects cylinder below edge bottom.
      var a = dx * dx + dy * dy;
      var c = p1x * p1x + p1y * p1y - radius * radius;
      var k = p1x * dx + p1y * dy;
      var discr = k * k - a * c;

      if (discr <= Math.EPSILON) { // Line doesn't intersect bottom circle.
        return false;
      }

      var discrSqrt = Math.sqrt(discr);
      var t3 = (-k - discrSqrt) / a; // Line and bottom circle intersection closest to start point.

      if (t3 >= 0.0) { 
        if (t3 < 1.0) { // Intersection is inside of the segment.
          output.fraction = t3;
          output.normal = p.set(p1x + t3 * dx, p1y + t3 * dy); // Will be coppied in c# wrapper.
          output.normal.mul(1.0 / radius);
        } else { // Intersection is after segment ends.
          return false;
        }
      } else { // The ray is pointing away from the line and bottom circle first intersection.
        var t4 = (-k + discrSqrt) / a; 
        var y4 = p1y + t4 * dy;

        if (y4 <= 0.0) { // Line and bottom circle intersection happens on the shape surface.
          if (t4 <= 0.0) { // Start point is outside of semicircle.
            return false;
          }
          // Start point is inside of semicircle.
        } else { // Line intersects top semicircle of the bottom circle.
          var y2 = p1y + t2 * dy; // Line and cylinder second intersection point.
          if (y2 > height) { // Line intersects cylinder above edge bottom, i. e. intersects top circle.

            c -= height * (2 * p1y - height);
            k -= dy * height;
            discr = k * k - a * c;

            if (discr <= 0.0) { // Line doesn't intersect top circle or has single intersection point, that should never happen.
              return null;
            }

            discrSqrt = Math.sqrt(discr);
            t4 = (-k + discrSqrt) / a;

            if (t4 <= 0.0) { // The ray is pointing away from top circle, no intersections.
              return false;
            }
            // The ray starts inside of the shape.
          } // Line intersects cylinder inside of the shape.
        }
        output.fraction = 0.0;
        output.normal = Vec2.ZERO; // Will be coppied in c# wrapper.
      }
    } else if (t1 >= 0.0) { // The ray intersects cylinder between top and bottom (first intersection point).
      output.fraction = t1;
      output.normal = p.set(dx > 0.0 ? -1.0 : 1.0, 0.0); // Will be coppied in c# wrapper.
    } else { // Start point is inside of cylinder and the ray is pointing away from first intersection point.
      var y2 = p1y + t2 * dy; // Line and cylinder second intersection point.

      if (y2 > height) { // Line intersects cylinder above edge top.
        var a = dx * dx + dy * dy;
        var c = p1x * p1x + (p1y - height) * (p1y - height) - radius * radius;
        var k = p1x * dx + dy * (p1y - height);
        var discr = k * k - a * c;

        if (discr <= 0.0) { // Line doesn't intersect top circle or has single intersection point, that should never happen.
          return false;
        }

        var discrSqrt = Math.sqrt(discr);
        var t4 = (-k + discrSqrt) / a; // Line and top circle intersection point, the most distant from the start point.

        if (t4 <= 0.0) { // The ray is pointing away from the top circle.
          return false;
        }
      } else if (y2 < 0.0) { // Line intersects cylinder below edge bottom.
        var a = dx * dx + dy * dy;
        var c = p1x * p1x + p1y * p1y - radius * radius;
        var k = p1x * dx + p1y * dy;
        var discr = k * k - a * c;

        if (discr <= 0.0) { // Line doesn't intersect bottom circle or has single intersection point, that should never happen.
          return false;
        }

        var discrSqrt = Math.sqrt(discr);
        var t4 = (-k + discrSqrt) / a; // Line and bottom circle intersection point, the most distant from the start point.

        if (t4 <= 0.0) { // The ray is pointing away from the bottom circle.
          return false;
        }
      }
      // The ray starts inside of the shape.
      output.fraction = 0.0;
      output.normal = Vec2.ZERO;
    }
  }

  output.normal.rotT(this.m_alignRot).rot(xf.q);
  return true;
};

EdgeShape.prototype.computeAABB = function(aabb, xf, childIndex) {
  var v1 = Transform.mulVec2(xf, this.m_vertex1);
  var v2 = Transform.mulVec2(xf, this.m_vertex2);

  aabb.combinePoints(v1, v2);
  aabb.extend(this.m_radius);
}

EdgeShape.prototype.computeMass = function(massData, density) {
  massData.mass = 0.0;
  massData.center.setCombine(0.5, this.m_vertex1, 0.5, this.m_vertex2);
  massData.I = 0.0;
}

EdgeShape.prototype.computeDistanceProxy = function(proxy) {
  proxy.m_vertices.push(this.m_vertex1);
  proxy.m_vertices.push(this.m_vertex2);
  proxy.m_count = 2;
  proxy.m_radius = this.m_radius;
};
