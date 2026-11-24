// One trajectory for the fire station and the battlefield.
// Samples along the shot, then bisects the first return to height 0.
// No eval or Function is used. The caller supplies the height at a distance.
(function (root) {
  var SAMPLE_STEP = 0.1;
  var MAX_METERS = 100;

  function bisectRoot(evalAt, lo, yLo, hi, yHi) {
    // lo is still above the ground, hi is on or below it.
    for (var k = 0; k < 52; k++) {
      var mid = (lo + hi) / 2;
      var yMid = evalAt(mid);
      if (!isFinite(yMid)) return null;
      if (yMid > 0) {
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

  function insertPeak(points, peakX, peakY) {
    if (!(peakX > 0)) return;
    for (var i = 0; i < points.length; i++) {
      if (Math.abs(points[i].x - peakX) < 1e-6) {
        if (peakY > points[i].y) points[i].y = peakY;
        return;
      }
      if (points[i].x > peakX) {
        points.splice(i, 0, { x: peakX, y: peakY });
        return;
      }
    }
  }

  // Height samples from distance 0 to the impact, or to maxDistance when the
  // shot never comes back to the ground. The last point of an impact is exactly
  // (distance, 0). points are spaced by sampleStep, with the peak inserted.
  function traceShell(evalAt, options) {
    var maxDistance = options && options.maxDistance > 0 ? options.maxDistance : MAX_METERS;
    var step = options && options.sampleStep > 0 ? options.sampleStep : SAMPLE_STEP;
    var speed = options && options.shellSpeed > 0 ? options.shellSpeed : 20;
    var y0 = evalAt(0);
    if (!isFinite(y0)) return null;

    var peak = y0;
    var peakX = 0;
    var leftGround = y0 > 0.02;
    var prevX = 0;
    var prevY = y0;
    var arc = 0;
    var impactX = null;
    var steps = Math.ceil(maxDistance / step);

    for (var i = 1; i <= steps; i++) {
      var x = Math.min(maxDistance, i * step);
      if (x <= prevX + 1e-9) break;
      var y = evalAt(x);
      if (!isFinite(y)) return null;
      if (y > peak) {
        peak = y;
        peakX = x;
      }
      if (y > 0.02) leftGround = true;
      if (leftGround && y <= 0) {
        impactX = bisectRoot(evalAt, prevX, prevY, x, y);
        if (impactX === null || !isFinite(impactX)) return null;
        arc += Math.hypot(impactX - prevX, -prevY);
        break;
      }
      arc += Math.hypot(x - prevX, y - prevY);
      prevX = x;
      prevY = y;
    }

    var end = impactX === null ? maxDistance : impactX;
    var refined = refinePeak(evalAt, peakX, peak, step, end);
    peakX = refined.x;
    peak = refined.y;

    var points = [];
    var count = Math.ceil(end / step);
    for (var j = 0; j <= count; j++) {
      var sx = j * step;
      if (sx >= end - 1e-7) break;
      var sy = evalAt(sx);
      if (!isFinite(sy)) return null;
      points.push({ x: sx, y: sy });
    }
    if (impactX !== null) points.push({ x: impactX, y: 0 });
    else {
      var yEnd = evalAt(end);
      if (!isFinite(yEnd)) return null;
      points.push({ x: end, y: yEnd });
    }
    insertPeak(points, peakX, peak);

    return {
      impact: impactX !== null,
      distance: impactX,
      peak: peak,
      peakX: peakX,
      time: arc / speed,
      points: points
    };
  }

  root.GraphHowitzerFire = {
    traceShell: traceShell,
    SAMPLE_STEP: SAMPLE_STEP
  };
})(typeof globalThis !== "undefined" ? globalThis : this);
