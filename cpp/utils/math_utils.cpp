float dot(std::array<float,3> a, std::array<float,3> b){
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void normalise(std::array<float, 3>& vec) {
    float norm = sqrt(dot(vec, vec)) + 1e-15;
    vec[0] /= norm;
    vec[1] /= norm;
    vec[2] /= norm;
}