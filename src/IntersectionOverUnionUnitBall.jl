module IntersectionOverUnionUnitBall

using GeometryTypes: HyperRectangle, Vec
using Roots: find_zero, Bisection

measure(bb::HyperRectangle) = prod(bb.widths)

function iou(bb1::HyperRectangle, bb2::HyperRectangle)
  measure_i = measure(bb1 ∩ bb2)
  measure_u = measure(bb1) + measure(bb2) - measure_i
  return measure_i / measure_u
end

"""
linear interpolation in parameter space
"""
function interpolate(bb1::HyperRectangle, bb2::HyperRectangle, α)
  origin = (1-α) * bb1.origin + α * bb2.origin
  widths = (1-α) * bb1.widths + α * bb2.widths
  return HyperRectangle(origin, widths)
end

bb_reference = HyperRectangle(Vec(0.0, 0.0), Vec(1.0, 1.0))

function random_bb()
  possible_origins = HyperRectangle(Vec(0.0, 0.0), Vec(5.0, 5.0))
  sampled_origin = 2 * (rand(Vec{2}) .- 0.5) .* possible_origins.widths + possible_origins.origin

  widths_max = 5 * (sampled_origin - bb_reference.origin)
  sampled_widths = rand(Vec{2}) .* widths_max
  return HyperRectangle(sampled_origin, sampled_widths)
end

function hunt_iou(ref_bb, other_bb, iou_goal)
  @assert iou_goal < 1
  family(α) = interpolate(ref_bb, other_bb, α)
  f(α) = iou(ref_bb, family(α)) - iou_goal
  # f(0) > 0 by construction
  @assert f(1) < 0
  α_star = find_zero(f, (0,1), Bisection())
  return family(α_star)
end

function random_bb_at_iou(ref_bb, iou_goal)
  while true
    rand_bb = random_bb()
    if iou(ref_bb, rand_bb) >= iou_goal
      continue
    end
    return hunt_iou(ref_bb, rand_bb, iou_goal)
  end
end

ball_samples(n) = [random_bb_at_iou(bb_reference, 0.5) for _=1:n]

end # module
