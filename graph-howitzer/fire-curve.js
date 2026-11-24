// One trajectory for the fire station and the battlefield.
// The line runs across the whole range, including below the ground.
// The impact is the first return to height 0. No eval or Function is used.
(function (root) {
  var SAMPLE_STEP = 0.1;
  var MAX_DISTANCE = 240;
  var MAX_ABS_HEIGHT = 200;

  function inBand(y, limit) {
    return isFinite(y) && Math.abs(y) <= limit;
  }

  // One end is inside the height band and the other has left it.
  // Returns the point on the boundary between them.
  function clipToBand(evalAt, xIn, yIn, xOut, yOut, limit) {
    for (var k = 0; k < 52; k++) {
      var mid = (xIn + xOut) / 2;
      var yMid = evalAt(mid);
      if (inBand(yMid, limit)) {
        xIn = mid;
        yIn = yMid;
      } else {
        xOut = mid;
        yOut = yMid;
      }
    }
    if (isFinite(yIn) && isFinite(yOut) && Math.abs(yOut) > limit && yOut !== yIn) {
      var target = (yOut > 0 ? 1 : -1) * limit;
      var t = (target - yIn) / (yOut - yIn);
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      return { x: xIn + (xOut - xIn) * t, y: target };
    }
    return { x: xIn, y: yIn };
  }

  function bisectRoot(evalAt, lo, yLo, hi, yHi) {
    for (var k = 0; k < 52; k++) {
      var mid = (lo + hi) / 2;
      var yMid = evalAt(mid);
      if (!isFinite(yMid)) return null;
      if (yMid === 0) return mid;
      var same = (yLo > 0 && yMid > 0) || (yLo < 0 && yMid < 0);
      if (same) {
        lo = mid;
        yLo = yMid;
      } else {
        hi = mid;
        yHi = yMid;
      }
    }
    var drop = yLo - yHi;
    var t = drop === 0 ? 1 : yLo / drop;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return lo + (hi - lo) * t;
  }

  function refinePeak(evalAt, peakX, peak, step, end) {
    var left = Math.max(0, peakX - step);
    var right = Math.min(end, peakX + step);
    if (!(right > left)) return { x: peakX, y: peak };
    for (var n = 0; n < 40; n++) {
      var m1 = left + (right - left) / 3;
      var m2 = right - (right - left) / 3;
      var f1 = evalAt(m1);
      var f2 = evalAt(m2);
      if (!isFinite(f1) || !isFinite(f2)) break;
      if (f1 < f2) left = m1;
      else right = m2;
    }
    var xPeak = (left + right) / 2;
    var yPeak = evalAt(xPeak);
    if (!isFinite(yPeak) || yPeak < peak) return { x: peakX, y: peak };
    return { x: xPeak, y: yPeak };
  }

  function insertPoint(points, x, y) {
    for (var i = 0; i < points.length; i++) {
      if (Math.abs(points[i].x - x) < 1e-4) {
        points[i].y = y;
        return;
      }
      if (points[i].x > x) {
        points.splice(i, 0, { x: x, y: y });
        return;
      }
    }
    points.push({ x: x, y: y });
  }

  // Above-ground runs and below-ground runs. A shared point sits on y = 0.
  function splitGround(points) {
    var above = [];
    var below = [];
    function add(runs, point, fresh) {
      if (fresh || !runs.length) runs.push([point]);
      else runs[runs.length - 1].push(point);
    }
    for (var i = 0; i < points.length; i++) {
      var p = points[i];
      var under = p.y < -1e-4;
      if (i === 0) {
        add(under ? below : above, p, true);
        continue;
      }
      var q = points[i - 1];
      var qUnder = q.y < -1e-4;
      if (qUnder === under) {
        add(under ? below : above, p, false);
        continue;
      }
      var denom = q.y - p.y;
      var t = denom === 0 ? 0 : q.y / denom;
      if (t < 0) t = 0;
      if (t > 1) t = 1;
      var z = { x: q.x + (p.x - q.x) * t, y: 0 };
      add(qUnder ? below : above, z, false);
      add(under ? below : above, z, true);
      add(under ? below : above, p, false);
    }
    return { above: above, below: below };
  }

  // Samples from 0 out to maxDistance. Stops cleanly where |height| leaves the
  // band or the value stops being finite. The first return to height 0 is the impact.
  function traceShell(evalAt, options) {
    var maxDistance = options && options.maxDistance > 0 ? options.maxDistance : MAX_DISTANCE;
    var maxAbs = options && options.maxAbsHeight > 0 ? options.maxAbsHeight : MAX_ABS_HEIGHT;
    var step = options && options.sampleStep > 0 ? options.sampleStep : SAMPLE_STEP;
    var speed = options && options.shellSpeed > 0 ? options.shellSpeed : 20;
    if (!(maxDistance > 0) || !(step > 0)) return null;

    var steps = Math.ceil(maxDistance / step);
    var points = [];
    var prevX = 0;
    var prevY = evalAt(0);
    var i = 0;
    var started = false;

    if (inBand(prevY, maxAbs)) {
      points.push({ x: 0, y: prevY });
      started = true;
    } else {
      for (i = 1; i <= steps; i++) {
        var x = Math.min(maxDistance, i * step);
        var y = evalAt(x);
        if (inBand(y, maxAbs)) {
          var entry = clipToBand(evalAt, x, y, prevX, prevY, maxAbs);
          points.push(entry);
          if (x > entry.x + 1e-4) points.push({ x: x, y: y });
          prevX = x;
          prevY = y;
          started = true;
          break;
        }
        prevX = x;
        prevY = y;
      }
      if (!started) return null;
    }

    var leftGround = Math.abs(points[0].y) > 0.02;
    var impactX = null;
    var arcHit = 0;
    var arcAll = 0;

    function consider(x0, y0, x1, y1) {
      var seg = Math.hypot(x1 - x0, y1 - y0);
      if (impactX === null) {
        var crosses = (y0 > 0 && y1 <= 0) || (y0 < 0 && y1 >= 0);
        if (leftGround && crosses && x1 > x0 + 1e-9) {
          var root = bisectRoot(evalAt, x0, y0, x1, y1);
          if (root !== null && isFinite(root)) {
            impactX = root;
            arcHit += Math.hypot(root - x0, -y0);
          } else {
            arcHit += seg;
          }
        } else {
          arcHit += seg;
        }
      }
      arcAll += seg;
      if (Math.abs(y1) > 0.02) leftGround = true;
    }

    for (var p = 1; p < points.length; p++) {
      consider(points[p - 1].x, points[p - 1].y, points[p].x, points[p].y);
    }

    for (i = i + 1; i <= steps; i++) {
      var nx = Math.min(maxDistance, i * step);
      if (nx <= prevX + 1e-9) break;
      var ny = evalAt(nx);
      if (!inBand(ny, maxAbs)) {
        var edge = clipToBand(evalAt, prevX, prevY, nx, ny, maxAbs);
        if (edge.x > prevX + 1e-6) {
          points.push(edge);
          consider(prevX, prevY, edge.x, edge.y);
        }
        break;
      }
      points.push({ x: nx, y: ny });
      consider(prevX, prevY, nx, ny);
      prevX = nx;
      prevY = ny;
    }

    if (!points.length) return null;

    var peak = points[0].y;
    var peakX = points[0].x;
    var limit = impactX === null ? Infinity : impactX + 1e-6;
    for (var k = 0; k < points.length; k++) {
      if (points[k].x > limit) break;
      if (points[k].y > peak) {
        peak = points[k].y;
        peakX = points[k].x;
      }
    }
    var end = impactX === null ? points[points.length - 1].x : impactX;
    var refined = refinePeak(evalAt, peakX, peak, step, end);
    peakX = refined.x;
    peak = refined.y;
    if (impactX !== null) insertPoint(points, impactX, 0);
    if (peakX > 0 && (impactX === null || peakX <= impactX + 1e-4)) insertPoint(points, peakX, peak);

    var high = 0;
    for (var h = 0; h < points.length; h++) {
      if (points[h].y > high) high = points[h].y;
    }

    return {
      impact: impactX !== null,
      distance: impactX,
      peak: peak,
      peakX: peakX,
      time: (impactX !== null ? arcHit : arcAll) / speed,
      points: points,
      high: high
    };
  }

  root.GraphHowitzerFire = {
    traceShell: traceShell,
    splitGround: splitGround,
    SAMPLE_STEP: SAMPLE_STEP,
    MAX_DISTANCE: MAX_DISTANCE,
    MAX_ABS_HEIGHT: MAX_ABS_HEIGHT
  };
})(typeof globalThis !== "undefined" ? globalThis : this);
