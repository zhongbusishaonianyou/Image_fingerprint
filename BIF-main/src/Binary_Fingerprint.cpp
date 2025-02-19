#include "binary_fingerprint.h"
#include <cmath>

/*using view_rim aligns Structural feature matrix (based on opencv)*/
inline cv::Mat1b circcolshift(const cv::Mat1b &src, int n)
{ 
        if (n == 0)
        return src.clone();

    cv::Mat1b dst(src.size(), src.type());
    src(cv::Range::all(), cv::Range(src.cols -n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
    src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
    return dst;
}

std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
} 

/**make Structural feature matrix from a frame of point cloud)*/
cv::Mat1b Binary_Image_Fingerprint::make_Structural_feature_matrix( const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    
    
     cv::Mat1b SFM=cv::Mat1b::zeros(num_rings, num_sectors);

     for (pcl::PointXYZ p : cloud.points)
     {
       float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
       
       if (dis>Max_range)
       continue;

        float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 24;
        float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
        int Q_dis = std::min(std::max((int)floor(dis), 0),num_rings-1);
        int Q_arc = std::min(std::max((int)floor(arc / 3.4f), 0), 7);
        int Q_yaw = std::min(std::max((int)floor((yaw + 0.5)/3.0), 0), num_sectors-1);
        SFM.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
      
      }
    
    return SFM;
} 
/*make viewrim align vector*/
cv::Mat1b Binary_Image_Fingerprint::makeViewrim(cv::Mat1b & sfm)
{
    
      cv::Mat1b Viewrim(1,num_sectors);
      cv::Mat1b curr_col(num_rings,1); 
      
    for ( int col_idx = 0; col_idx < num_sectors; col_idx++ )
    {
         curr_col = sfm.col(col_idx);

         for ( int row_idx =num_rings-1; row_idx > -1; row_idx-- )
       {       
        if(curr_col(row_idx,0)!=0)
        {
         Viewrim(0,col_idx)=row_idx;
         break;
        }
       }
    }
   // cout<<Viewrim<<endl;
     
    return Viewrim;
} 

int Binary_Image_Fingerprint::Align_Viewrim( cv::Mat1b  & viewrim1, cv::Mat1b  & viewrim2)
{  
    
    int argmin_rim_shift = 0;
    double min_rim_diff = 100000;
    for ( int shift_idx = 0; shift_idx < num_sectors; shift_idx++ )
    {   
        
        cv::Mat1b rim2_shifted = circcolshift(viewrim2, shift_idx);
        
        cv::Mat1b  rim_diff = cv::abs(viewrim1 - rim2_shifted);
       
        int cur_diff=cv::sum(rim_diff)[0];
        
        if( cur_diff < min_rim_diff )
        {
            argmin_rim_shift = shift_idx;
            min_rim_diff = cur_diff;
        }
    }
    return argmin_rim_shift;

} 

/*Hamming distance between image fingerprints*/
double Binary_Image_Fingerprint::calculate_hamming_dis(cv::Mat1b &Fingerprint1,cv::Mat1b &Fingerprint2 )
{   
    cv::Mat1b hamming=cv::Mat1b::zeros(FINGERPRINT_SIZE,FINGERPRINT_SIZE);
    cv::bitwise_xor(Fingerprint1,Fingerprint2,hamming);
    
    double dis_hamming = cv::sum(hamming)[0]/256;
   
    return dis_hamming;

} 

/*Hamming distance between image fingerprints*/
std::pair<double, int> Binary_Image_Fingerprint::similarity_score( cv::Mat1b &curr_bif, cv::Mat1b & curr_viewrim,cv::Mat1b &des2,cv::Mat1b &candi_bif,cv::Mat1b & candi_viewrim )
{
   // clock_t startTime = clock(); 
    double hamming_dist;

    int argmin_rim_shift = Align_Viewrim( curr_viewrim, candi_viewrim );

   

        if (argmin_rim_shift==0)
    {
         hamming_dist = calculate_hamming_dis(curr_bif,candi_bif);
    }
    else
    {
   
     cv::Mat1b des2_shifted = circcolshift(des2, argmin_rim_shift);
     cv::Mat1b fingerprint2 =  makeBinaryfingerprint(des2_shifted);
     hamming_dist = calculate_hamming_dis(curr_bif,fingerprint2);
    }
 
    //  clock_t endTime = clock();
   // cout << "times = " << (endTime - startTime) / (double)CLOCKS_PER_SEC << "s."<< endl;
    return make_pair(hamming_dist, argmin_rim_shift);

} 

cv::Mat1b Binary_Image_Fingerprint::makeBinaryfingerprint(cv::Mat1b & des)
{    
    
    double aver; cv::Mat des1;
    cv::Mat trans_img;
    cv::Mat hash;
    cv::Mat fingerprint;
    
    des.convertTo(des1,CV_32F,1.0/255);

    cv::dct(des1,trans_img);
    
    cv::Rect rect(0,0, FINGERPRINT_SIZE, FINGERPRINT_SIZE);
    
    hash=trans_img(rect);
    aver=cv::mean(hash)[0];
    
    cv::threshold(hash,fingerprint,aver,1,cv::THRESH_BINARY);
    
    return fingerprint;
}
MatrixXd Binary_Image_Fingerprint::makeRingkeys(cv::Mat1b & sfm)
{
   
   Eigen::MatrixXd Ringkey(num_rings, 1);
   Eigen::MatrixXi curr_row(1,num_sectors);
   MatrixXi sfm1(num_rings,num_sectors);

    cv2eigen(sfm,sfm1);
    for ( int row_idx = 0; row_idx < num_rings; row_idx++ )
    {
        curr_row = sfm1.row(row_idx);
        Ringkey(row_idx, 0) = curr_row.sum()/120.0f;
    }
   // cout <<Ringkey.size() << endl;
    return Ringkey;
} 


void Binary_Image_Fingerprint::makeAndSaveinfo( pcl::PointCloud<pcl::PointXYZ> & cloud )
{  
   
    cv::Mat1b sfm = make_Structural_feature_matrix(cloud);  
    Eigen::MatrixXd ringkey = makeRingkeys( sfm );
    cv::Mat1b viewrim = makeViewrim( sfm );
    cv::Mat1b fingerprint = makeBinaryfingerprint(sfm);
    std::vector<float> ringkey_vec = eig2stdvec( ringkey );

    SFM.push_back( sfm ); 
    Ring_keys.push_back( ringkey );
    View_Rims.push_back( viewrim );
    BIF.push_back(fingerprint);
    Ringkeys_mat.push_back( ringkey_vec );

} 

std::pair<int, float> Binary_Image_Fingerprint::detectLoopClosureID ( void )
{

    int loop_id { -1 }; 

    auto curr_key = Ringkeys_mat.back(); 
    auto curr_bif = BIF.back(); 
    auto curr_viewrim=View_Rims.back();
  
    // tree_ reconstruction (not mandatory to make everytime)
    if( tree_making_period_counter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {
        TicToc t_tree_construction;

        Ringkeys_to_search_.clear();
        Ringkeys_to_search_.assign( Ringkeys_mat.begin(), Ringkeys_mat.end() - NUM_EXCLUDE_RECENT ) ;

        Ringkeys_tree_.reset(); 
        Ringkeys_tree_ = std::make_unique<InvKeyTree>(num_rings, Ringkeys_to_search_, NUM_CANDIDATES_FROM_TREE  );
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
        t_tree_construction.toc("Tree construction");
    }
    tree_making_period_counter = tree_making_period_counter + 1;
        
    float min_dist = 1000; // init with somthing large
    int matching_idx = -1;

    // knn search
    std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
    std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

    TicToc t_tree_search;
    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    Ringkeys_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 
    t_tree_search.toc("Tree search");
   
    /* 
     *  step 2: pairwise distance(find optimal columnwise best-fit using viewrim)
     */
    TicToc t_calc_dist; 
    
    
    for ( int candidate_idx = 0; candidate_idx < NUM_CANDIDATES_FROM_TREE; candidate_idx++ )
    {
        cv::Mat1b Ringkeys_candidate = SFM[ candidate_indexes[candidate_idx] ];
        cv::Mat1b bif_candidate = BIF[ candidate_indexes[candidate_idx] ];
        cv::Mat1b view_candidate = View_Rims[ candidate_indexes[candidate_idx] ];

        std::pair<double, int> hammming_dist = similarity_score( curr_bif, curr_viewrim,Ringkeys_candidate,bif_candidate,view_candidate); 
        
        double candidate_dist = hammming_dist.first;
        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            matching_idx = candidate_indexes[candidate_idx];
        }
        
    }
     
     t_calc_dist.toc("Distance calc");
    
     loop_id = matching_idx+1; 
    
    //     std::cout.precision(3); 
    //     cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts.size()-1 << " and " << nn_idx << "." << endl;
    //     cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    // }
    std::pair<int, float> result {loop_id, min_dist};
   
    return result;

} 
