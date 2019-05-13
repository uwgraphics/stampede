
function get_relaxedik_path(path_to_src)
    fp = path_to_src * "/Stampede/Config/relaxedik_path"
    try
        f = open(fp, "r")
        line = readline(f)
        return line * "/src"
    catch
        println("relaxed_ik path not found.  Please run the create_relaxedik_path script and try again:
        \n
            rosrun stampede create_relaxedik_path.py
        \n")
    end
end
