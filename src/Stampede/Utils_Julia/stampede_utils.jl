using Distributions

function get_bounded_rand_state(relaxedIK_vars)
    sample = Array{Float64,1}()
    bounds = relaxedIK_vars.vars.bounds
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end
