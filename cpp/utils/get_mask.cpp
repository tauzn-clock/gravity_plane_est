std::vector<int> get_mask(
    std::vector<std::array<float,3> >& normal, 
    std::vector<std::array<float,3> > points,
    std::array<float,3> grav,
    float bound,
    int kernel_size,
    int cluster_size
){
    std::vector<float> dist(normal.size());

    centre_to_hemisphere(normal,grav);

    float largest = -10000;
    float smallest = 10000;

    for(int i=0; i<normal.size(); ++i){
        if (dot(normal[i], grav)>bound){
            dist[i] = dot(points[i], grav);
            if (largest<dist[i]) largest = dist[i];
            if (smallest>dist[i]) smallest = dist[i];
        }
    }

    //Cluster in cm bins
    float CLUSTER_SIZE = 0.01;
    int INV_CLUSTER_SIZE = (int)(1/CLUSTER_SIZE);

    std::vector<int> bins((int)((largest-smallest)*INV_CLUSTER_SIZE)+1);

    for(int i=0; i<dist.size(); i++){
        if (dist[i]!=0){
            int index_i = (int)((dist[i]-smallest)*INV_CLUSTER_SIZE);
            bins[index_i]++;
        }
    }

    std::vector<int> dillation(bins.size());
    for(int i=0; i<dillation.size(); i++){
        for(int j=std::max(i-kernel_size/2, 0); j<std::min(i+kernel_size/2+1,(int)dillation.size()); j++){
            dillation[i] = std::max(dillation[i], bins[j]);
        }
    }

    std::vector< std::pair<int,int> > store_index;
    for(int i=0; i<dillation.size(); i++){
        if (bins[i]==dillation[i] && bins[i]!=0){
            store_index.push_back(std::make_pair(i,bins[i]));
        }
    }

    int PLANE_CNT = 4;

    std::partial_sort(
        store_index.begin(), store_index.begin() + std::min((int)store_index.size(),PLANE_CNT), store_index.end(),
        [](const std::pair<int,int>& a, const std::pair<int,int>& b) {
            return a.second > b.second;  // Sort in descending order
        }
    );

    std::cout<<"prop:"<<" "<<dillation.size()<<" "<<store_index.size()<<std::endl;
    for(int i=0; i<store_index.size(); i++){
        std::cout<<store_index[i].first<<" "<<store_index[i].second<<std::endl;
    }

    std::vector<int> mask(normal.size());

    for(int i=0; i<mask.size(); i++){
        if (dist[i]!=0){
            int index_i = (int)((dist[i]-smallest)*INV_CLUSTER_SIZE);
            for(int j=0; j<std::min((int)store_index.size(),PLANE_CNT); j++){
                if (index_i>=store_index[j].first-kernel_size/2 && index_i<=store_index[j].first+kernel_size/2){
                    mask[i] = j;
                    break;
                }
            }
        }
    }

    return mask;
}