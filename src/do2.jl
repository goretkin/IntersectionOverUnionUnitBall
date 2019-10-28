using Plots
using Printf
import IntersectionOverUnionUnitBall
using IntersectionOverUnionUnitBall: iou_ball, bb_reference, center, iou
using GeometryTypes: Point, HyperRectangle, Vec
using PolygonMinkowskiSum: SimplePolygon, minkowski_sum
using StructArrays

Plots.pyplot()

# maybe broadcast (.+) is more appropriate
function Base.:+(point::Point, poly::SimplePolygon)
    SimplePolygon(Ref(point) .+ poly.points)
end

function make_once()
    once_happened = nothing

    function once(fig_ij, label)
        if once_happened === nothing
            once_happened = fig_ij
        end
        if once_happened == fig_ij label else () end
    end
end



function make_frame(;
    subfigure_center_spacing = 2.6,
    iou_radius = 0.4,
    n_grid_square = 5,
    n_grid_extra = 3
)

    n_grid = n_grid_square + n_grid_extra
    lim = (0, subfigure_center_spacing * (n_grid + 1))


    biggest_square_length = sqrt(1/iou_radius) - 100*eps()
    smallest_square_length = sqrt(iou_radius) + 100*eps()

    widths_range_sq = range(smallest_square_length, biggest_square_length, length=n_grid_square)
    Δ = minimum(diff(widths_range_sq))

    widths_range = vcat(
        widths_range_sq,
        biggest_square_length .+ (1:n_grid_extra) .* Δ
    )

    ticks = [(pos=subfigure_center_spacing * i, lab=(@sprintf "%0.2f" w)) for (i, w) in enumerate(widths_range)]
    sa_ticks = StructArray(ticks)
    ticks_arg = (sa_ticks.pos, sa_ticks.lab)

    p = Plots.plot(;
        xlim=lim, ylim=lim,
        xticks=ticks_arg, yticks=ticks_arg,
        title=(@sprintf "iou= %0.2f" iou_radius), aspect_ratio=1, legend=false)

    once = make_once()

    for (fig_i, w) = enumerate(widths_range), (fig_j, h) = enumerate(widths_range)
        fig_ij = (fig_i, fig_j)
        other_widths = Vec(w, h)
        fig_center = Point((subfigure_center_spacing .* fig_ij)...)


        sweepme = HyperRectangle(Vec(0, 0) - other_widths/2, other_widths)
        max_iou = iou(sweepme, bb_reference)
        if max_iou <= iou_radius
            # position of `sweepme` has enough iou with bb_reference
            continue
        end

        center_path = map(x->Point(center(x)...), iou_ball(iou_radius, bb_reference, other_widths, 200))
        outer = minkowski_sum(SimplePolygon(sweepme), SimplePolygon(center_path))

        Plots.plot!(p, fig_center + SimplePolygon(bb_reference);
            fillalpha=0.2, fillcolor=:blue, linecolor=:gray, once(fig_ij, (label="reference",))...)

        example_p = center_path[1]
        example = HyperRectangle(sweepme.origin + example_p, sweepme.widths)
        Plots.plot!(p, fig_center + SimplePolygon(example);
            fillalpha=0.2, fillcolor=:green, linecolor=:gray, once(fig_ij, (label="example",))...)

        Plots.plot!(p, fig_center + SimplePolygon(center_path);
            fillalpha=0, linecolor=:black, linestyle=:solid, once(fig_ij, (label="center",))...)
        Plots.plot!(p, fig_center + outer;
            fillalpha=0, linecolor=:red, linestyle=:dot, once(fig_ij, (label="outer",))...)
        #=
        Plots.scatter!(p, [fig_center + example_p];
            color=:green, once(fig_ij, (label="example",))...)
        =#
    end
    return p
end

path_frames = abspath(joinpath(pathof(IntersectionOverUnionUnitBall), "../../build/frames"))
mkpath(path_frames)
for (i, iou_radius) = enumerate(range(0.5, 0.9, step=0.01))
    @show i, iou_radius
    i_str = @sprintf "%04d" i
    p = make_frame(iou_radius=iou_radius)
    path_frame = joinpath(path_frames, "$(i_str).png")
    Plots.savefig(p, path_frame)
end

display(p)
