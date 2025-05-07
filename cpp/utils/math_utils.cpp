float dot(std::array<float,3> a, std::array<float,3> b){
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void normalise(std::array<float, 3>& vec) {
    float norm = sqrt(dot(vec, vec)) + 1e-15;
    vec[0] /= norm;
    vec[1] /= norm;
    vec[2] /= norm;
}

void rotation(std::array<float,3>& vec, float alpha, float beta, float gamma){
    vec[0] = vec[0] * (cos(beta) * cos(gamma)) + vec[1] * (-cos(beta) * sin(gamma)) + vec[2] * (sin(beta));
    vec[1] = vec[0] * (cos(alpha) * sin(gamma) + sin(alpha) * sin(beta) * cos(gamma)) + vec[1] * (cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma)) + vec[2] * (-sin(alpha) * cos(beta));
    vec[2] = vec[0] * (-sin(alpha) * sin(gamma) + cos(alpha) * sin(beta) * cos(gamma)) + vec[1] * (-sin(alpha) * cos(gamma) - cos(alpha) * sin(beta) * sin(gamma)) + vec[2] * (cos(alpha) * cos(beta));
}