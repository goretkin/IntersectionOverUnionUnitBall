module PlotsGeometryTypes

using Plots: Shape
using GeometryTypes: HyperRectangle, Point
using PolygonMinkowskiSum: SimplePolygon

function Shape(r::HyperRectangle{2})
  (w, h) = r.widths
  (x, y) = r.origin
  Shape(x .+ [0,w,w,0, 0], y .+ [0,0,h,h, 0])
end

# TODO should be defined elsewhere
function SimplePolygon(r::HyperRectangle{2})
  # ensure order is ccw
  (w, h) = r.widths
  (x, y) = r.origin
  (xp, yp) = (x .+ [0,w,w,0], y .+ [0,0,h,h])
  # TODO static array length 4
  SimplePolygon([Point(x,y) for (x,y) in zip(xp, yp)])
end

end
