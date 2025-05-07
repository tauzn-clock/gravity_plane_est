std::vector<int> get_mask(
    std::vector<std::array<float,3> >& normal, 
    std::vector<std::array<float,3> > points,
    std::array<float,3> grav,
    float bound,
    int kernel_size,
    int cluster_size
){
    std::vector<int> mask(normal.size());
    std::vector<float> dist(normal.size());

    centre_to_hemisphere(normal,grav);

    for(int i=0; i<normal.size(); ++i){
        if (dot(normal[index], grav)<bound){
            dist[index] = dot(points[index, grav]);
        }
    }

    return mask;
}