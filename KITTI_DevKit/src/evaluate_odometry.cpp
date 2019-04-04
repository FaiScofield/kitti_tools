#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>

#include "mail.h"
#include "matrix.h"

using namespace std;

// static parameter
float lengths[] = {100,200,300,400,500,600,700,800};
int32_t num_lengths = 8;

struct errors {
    int32_t first_frame;
    int32_t last_frame;
    float   r_err;
    float   t_err;
    float   len;
    float   speed;

    errors (int32_t first_frame, int32_t last_frame, float r_err, float t_err, float len, float speed) :
        first_frame(first_frame), last_frame(last_frame), r_err(r_err), t_err(t_err), len(len), speed(speed) {}
};

vector<Matrix> loadPoses(string file_name) {
    cout << "Reading poses from " << file_name << endl;
    vector<Matrix> poses;
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp) return poses;
    int line = 1;
    while (!feof(fp)) {
        Matrix P = Matrix::eye(4);
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P.val[0][0], &P.val[0][1], &P.val[0][2], &P.val[0][3],
                   &P.val[1][0], &P.val[1][1], &P.val[1][2], &P.val[1][3],
                   &P.val[2][0], &P.val[2][1], &P.val[2][2], &P.val[2][3]) == 12) {
            poses.push_back(P);
        } else {
            cerr << "Warning: skip line " << line << " because it's a not-full-pose or it reached the end." << endl;
        }
        line++;
    }
    fclose(fp);
    return poses;
}

vector<float> trajectoryDistances (vector<Matrix> &poses) {
    vector<float> dist;
    dist.push_back(0);
    for (int32_t i=1; i<poses.size(); i++) {
        Matrix P1 = poses[i-1];
        Matrix P2 = poses[i];
        double dx = P1.val[0][3]-P2.val[0][3];
        double dy = P1.val[1][3]-P2.val[1][3];
        double dz = P1.val[2][3]-P2.val[2][3];
        dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
    }
    return dist;
}

int32_t lastFrameFromSegmentLength(vector<float> &dist, int32_t first_frame, float len) {
    for (int32_t i=first_frame; i<dist.size(); i++)
        if (dist[i]>dist[first_frame]+len)
            return i;
    return -1;
}

// pose_error = [ R_re.inv * R_gt  R_re.inv*(t_gt + t_re) ]
//              [         0                  1            ]
// 旋转误差越小，R_re.inv * R_gt 的值越接近I_3，则对角线上的元素和接近3，d值接近1，返回值接近0
// 旋转误差越小，d值越小(可能为负)，返回值接近0
inline float rotationError(Matrix &pose_error) {
    float a = pose_error.val[0][0];
    float b = pose_error.val[1][1];
    float c = pose_error.val[2][2];
    float d = 0.5*(a+b+c-1.0);
    return acos(max(min(d,1.0f), -1.0f)); // 返回弧度
}

inline float translationError(Matrix &pose_error) {
    float dx = pose_error.val[0][3];
    float dy = pose_error.val[1][3];
    float dz = pose_error.val[2][3];
    return sqrt(dx*dx+dy*dy+dz*dz);
}

vector<errors> calcSequenceErrors (vector<Matrix> &poses_gt,vector<Matrix> &poses_result) {
    // error vector
    vector<errors> err;

    // parameters
    int32_t step_size = 10;

    // pre-compute distances (from ground truth as reference)
    vector<float> dist = trajectoryDistances(poses_gt);

    // for all start positions do
    for (int32_t first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {
        // for all segment lengths do
        for (int32_t i=0; i<num_lengths; i++) {

            // current length
            float len = lengths[i];

            // compute last frame
            // first_frame和last_frame间距离相差lengths[i]米(100,200,...)
            int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

            // continue, if sequence not long enough
            if (last_frame == -1) {
//                continue; ?
                break;
            }

            // compute rotational and translational errors
            Matrix pose_delta_gt     = Matrix::inv(poses_gt[first_frame])*poses_gt[last_frame];
            Matrix pose_delta_result = Matrix::inv(poses_result[first_frame])*poses_result[last_frame];
            Matrix pose_error        = Matrix::inv(pose_delta_result)*pose_delta_gt;
            float r_err = rotationError(pose_error);
            float t_err = translationError(pose_error);

            // compute speed
            float num_frames = (float)(last_frame-first_frame+1);
            float speed = len/(0.1*num_frames); // 数据都是10帧1秒

            // write to file
            err.push_back(errors(first_frame, last_frame, r_err/len, t_err/len, len, speed));
        }
    }

    printf("Tatal distance: %f\n", dist.back());

    // return error vector
    return err;
}

void saveSequenceErrors (vector<errors> &err, string file_name) {
    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(), "w");

    // write to file
//    float t_err_sum = 0.0;
    for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
        fprintf(fp, "%d %d %f %f %f %f\n", it->first_frame, it->last_frame, it->r_err, it->t_err, it->len, it->speed);
//        t_err_sum += it->t_err;
    }

//    printf("Root Mean Square Error(RMSE):%f\n", t_err_sum/err.size());
//    fprintf(fp, "RMSE: %f\n", t_err_sum/err.size());

    // close file
    fclose(fp);
}

void savePathPlot (vector<Matrix> &poses_gt, vector<Matrix> &poses_result, string file_name) {
    // parameters
    int32_t step_size = 5;

    // open file
    FILE *fp = fopen(file_name.c_str(), "w");

    // save x/z coordinates of all frames to file
    for (int32_t i=0; i<poses_gt.size(); i+=step_size)
    fprintf(fp, "%f %f %f %f\n", poses_gt[i].val[0][3], poses_gt[i].val[2][3],
            poses_result[i].val[0][3], poses_result[i].val[2][3]);

    // close file
    fclose(fp);
}

// 计算绘图坐标轴的尺度和边界
vector<int32_t> computeRoi (vector<Matrix> &poses_gt, vector<Matrix> &poses_result) {
    float x_min = numeric_limits<int32_t>::max();
    float x_max = numeric_limits<int32_t>::min();
    float z_min = numeric_limits<int32_t>::max();
    float z_max = numeric_limits<int32_t>::min();

    for (vector<Matrix>::iterator it=poses_gt.begin(); it!=poses_gt.end(); it++) {
        float x = it->val[0][3];
        float z = it->val[2][3];
        if (x < x_min) x_min = x; if (x > x_max) x_max = x;
        if (z < z_min) z_min = z; if (z > z_max) z_max = z;
    }

    for (vector<Matrix>::iterator it=poses_result.begin(); it!=poses_result.end(); it++) {
        float x = it->val[0][3];
        float z = it->val[2][3];
        if (x < x_min) x_min = x; if (x > x_max) x_max = x;
        if (z < z_min) z_min = z; if (z > z_max) z_max = z;
    }

    float dx = 1.1*(x_max-x_min);   //绘图的总尺度
    float dz = 1.1*(z_max-z_min);
    float mx = 0.5*(x_max+x_min);   //尺度中值
    float mz = 0.5*(z_max+z_min);
    float r  = 0.5*max(dx,dz);

    vector<int32_t> roi;
    roi.push_back((int32_t)(mx - r));   // 坐标轴左边界
    roi.push_back((int32_t)(mx + r));   // 坐标轴右边界
    roi.push_back((int32_t)(mz - r));   // 坐标轴下边界
    roi.push_back((int32_t)(mz + r));   // 坐标轴上边界

    return roi;
}

void plotPathPlot (string dir,vector<int32_t> &roi,int32_t sequence) {
    // gnuplot file name
    char command[1024];
    char file_name[256];
    sprintf(file_name, "%02d.gp", sequence);
    string full_name = dir + "/" + file_name;

    // open file
    FILE *fp = fopen(full_name.c_str(), "w");

    // save gnuplot instructions
    fprintf(fp, "set term png size 900,900\n");
    fprintf(fp, "set output \"%02d.png\"\n", sequence);
    fprintf(fp, "set size ratio -1\n");
    fprintf(fp, "set xrange [%d:%d]\n", roi[0], roi[1]);
    fprintf(fp, "set yrange [%d:%d]\n", roi[2], roi[3]);
    fprintf(fp, "set xlabel \"x [m]\"\n");
    fprintf(fp, "set ylabel \"y [m]\"\n");
    fprintf(fp, "plot \"%02d.txt\" using 1:2 with lines lc rgb \"#FF0000\" lw 2 title 'Ground Truth',", sequence);
    fprintf(fp, "\"%02d.txt\" using 3:4 with lines lc rgb \"#0000FF\" lw 2 title 'Odometry' ,", sequence);
    fprintf(fp, "\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' with points\n",sequence);

    // close file
    fclose(fp);

    // run gnuplot => create png + eps
    sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
    system(command);
}

void saveErrorPlots(vector<errors> &seq_err,string plot_error_dir,char* prefix) {
    // file names
    char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix);
    char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix);
    char file_name_ts[1024]; sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix);
    char file_name_rs[1024]; sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix);

    // open files
    FILE *fp_tl = fopen(file_name_tl,"w");
    FILE *fp_rl = fopen(file_name_rl,"w");
    FILE *fp_ts = fopen(file_name_ts,"w");
    FILE *fp_rs = fopen(file_name_rs,"w");

    // for each segment length do
    for (int32_t i=0; i<num_lengths; i++) {
        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->len - lengths[i]) < 1.0) { // 用<1.0代替==
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_tl, "%f %f\n", lengths[i], t_err/num);
            fprintf(fp_rl, "%f %f\n", lengths[i], r_err/num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed=2; speed<25; speed+=2) {
        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->speed - speed) < 2.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_ts, "%f %f\n", speed, t_err/num);
            fprintf(fp_rs, "%f %f\n", speed, r_err/num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void plotErrorPlots (string dir,char* prefix) {
    char command[1024];

    // for all four error plots do
    for (int32_t i=0; i<4; i++) {
        // create suffix
        char suffix[16];
        switch (i) {
          case 0: sprintf(suffix,"tl"); break;
          case 1: sprintf(suffix,"rl"); break;
          case 2: sprintf(suffix,"ts"); break;
          case 3: sprintf(suffix,"rs"); break;
        }

        // gnuplot file name
        char file_name[1024]; char full_name[1024];
        sprintf(file_name, "%s_%s.gp", prefix, suffix);
        sprintf(full_name, "%s/%s", dir.c_str(), file_name);

        // open file
        FILE *fp = fopen(full_name, "w");

        // save gnuplot instructions
        fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
        fprintf(fp,"set output \"%s_%s.png\"\n",prefix,suffix);

        // start plot at 0
        fprintf(fp,"set size ratio 0.5\n");
        fprintf(fp,"set yrange [0:*]\n");

        // x label
        if (i<=1) fprintf(fp,"set xlabel \"Path Length [m]\"\n");
        else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");

        // y label
        if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
        else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");

        // plot error curve
        fprintf(fp,"plot \"%s_%s.txt\" using ",prefix,suffix);
        switch (i) {
            case 0: fprintf(fp,"1:($2*100) title 'Translation Error'"); break;
            case 1: fprintf(fp,"1:($2*57.3) title 'Rotation Error'"); break;
            case 2: fprintf(fp,"($1*3.6):($2*100) title 'Translation Error'"); break;
            case 3: fprintf(fp,"($1*3.6):($2*57.3) title 'Rotation Error'"); break;
        }
        fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(command,"cd %s; gnuplot %s", dir.c_str(), file_name);
        system(command);
    }
}

// 保存最终的平均平移/旋转误差
void saveStats (vector<errors> err,string dir) {
    float t_err = 0;
    float r_err = 0;

    // for all errors do => compute sum of t_err, r_err
    for (vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
        t_err += it->t_err;
        r_err += it->r_err;
    }

    // open file
    FILE *fp = fopen((dir + "/stats.txt").c_str(),"w");

    // save errors
    float num = err.size();
    fprintf(fp,"%f %f\n", t_err/num, r_err/num);

    // close file
    fclose(fp);
}

// 评估所有11至21序列
bool eval (string result_sha,Mail* mail) {

  // ground truth and result directories
  string gt_dir         = "./data/odometry/poses";
  string result_dir     = "./results";
  string error_dir      = result_dir + "/errors";
  string plot_path_dir  = result_dir + "/plot_path";
  string plot_error_dir = result_dir + "/plot_error";

  // create output directories
  system(("mkdir " + error_dir).c_str());
  system(("mkdir " + plot_path_dir).c_str());
  system(("mkdir " + plot_error_dir).c_str());

  // total errors
  vector<errors> total_err;

  // for all sequences do
  for (int32_t i=11; i<22; i++) {

    // file name
    char file_name[256];
    sprintf(file_name,"%02d.txt",i);

    // read ground truth and result poses
    vector<Matrix> poses_gt     = loadPoses(gt_dir + "/" + file_name);
    vector<Matrix> poses_result = loadPoses(result_dir + "/data/" + file_name);

    // plot status
    mail->msg("Processing: %s, poses: %d/%d",file_name,poses_result.size(),poses_gt.size());

    // check for errors
//    if (poses_gt.size()==0 || poses_result.size() != poses_gt.size()) {
    if (poses_gt.size()==0) {
      mail->msg("ERROR: Couldn't read (all) poses of: %s", file_name);
      return false;
    }


    // compute sequence errors
    vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result);
    saveSequenceErrors(seq_err, error_dir + "/" + file_name);

    // add to total errors
    total_err.insert(total_err.end(),seq_err.begin(),seq_err.end());

    // for first half => plot trajectory and compute individual stats
    if (i<=15) {

      // save + plot bird's eye view trajectories
      savePathPlot(poses_gt,poses_result,plot_path_dir + "/" + file_name);
      vector<int32_t> roi = computeRoi(poses_gt,poses_result);
      plotPathPlot(plot_path_dir,roi,i);

      // save + plot individual errors
      char prefix[16];
      sprintf(prefix,"%02d",i);
      saveErrorPlots(seq_err,plot_error_dir,prefix);
      plotErrorPlots(plot_error_dir,prefix);
    }
  }

  // save + plot total errors + summary statistics
  if (total_err.size()>0) {
    char prefix[16];
    sprintf(prefix,"avg");
    saveErrorPlots(total_err,plot_error_dir,prefix);
    plotErrorPlots(plot_error_dir,prefix);
    saveStats(total_err,result_dir);
  }

  // success
    return true;
}

// 评估单个序列
bool eval2 (string path2ResultPoses_, int32_t sequence_, Mail* mail_) {

    // ground truth and result directories
    string gt_dir = "../poses";

    // create output directories
    char results_dir[256];
    sprintf(results_dir, "./results-%d", sequence_);
    string error_dir      = std::string(results_dir) + "/errors";
    string plot_path_dir  = std::string(results_dir) + "/plot_path";
    string plot_error_dir = std::string(results_dir) + "/plot_error";

    system(("rm -r " + std::string(results_dir)).c_str());
    if ( system(("mkdir " + std::string(results_dir)).c_str()) ) return false;
    if ( system(("mkdir " + error_dir).c_str()) ) return false;
    if ( system(("mkdir " + plot_path_dir).c_str()) ) return false;
    if ( system(("mkdir " + plot_error_dir).c_str()) ) return false;

    system(("cp " + path2ResultPoses_ + " " + std::string(results_dir)).c_str());

    // total errors
    vector<errors> total_err;

    // file name
    char file_name[256];
    sprintf(file_name, "%02d.txt", sequence_);

    // read ground truth and result poses
    vector<Matrix> poses_gt     = loadPoses(gt_dir + "/" + file_name);
    vector<Matrix> poses_result = loadPoses(path2ResultPoses_);

    // plot status
    mail_->msg("Processing: %s, tatal poses(result/gt): %d/%d", file_name, poses_result.size(), poses_gt.size());

    // check for errors
    if (poses_gt.size()==0) {
        mail_->msg("ERROR: Couldn't read (all) gt poses of: %s", file_name);
        return false;
    }
    if (poses_result.size()==0) {
        mail_->msg("ERROR: Couldn't read (all) result poses of: %s", file_name);
        return false;
    }

    // unequal size
    if (poses_result.size() < poses_gt.size()) {
        mail_->msg("Warning: poses size all not equal!");
        size_t ds = poses_gt.size() - poses_result.size();
    //      size_t ds_2 = ds * 0.6;
        mail_->msg("Erase gt %d poses from end!", ds);
        for (int i = 0; i < ds; ++i)
            poses_gt.pop_back();
    //      vector<Matrix> poses_tmp(poses_gt.begin()+ds_2, poses_gt.begin()+ds_2+poses_result.size());
    //      poses_gt.swap(poses_tmp);
    }
    if (poses_result.size() > poses_gt.size()) {
        mail_->msg("Warning: poses size all not equal!");
        size_t ds = poses_result.size() - poses_gt.size();
    //      size_t ds_2 = ds * 0.6;
        mail_->msg("Erase odom %d poses from end!", ds);
        for (int i = 0; i < ds; ++i)
            poses_result.pop_back();
    //      vector<Matrix> poses_tmp(poses_result.begin()+ds_2, poses_result.begin()+ds_2+poses_gt.size());
    //      poses_result.swap(poses_tmp);
    }
    std::cout << "now poses size: " << poses_result.size() << "/" << poses_gt.size() << std::endl;


    // compute sequence errors
    vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result);
    saveSequenceErrors(seq_err, error_dir + "/" + file_name);

    // add to total errors
    total_err.insert(total_err.end(), seq_err.begin(), seq_err.end());

    // for first half => plot trajectory and compute individual stats
    // save + plot bird's eye view trajectories
    savePathPlot(poses_gt, poses_result, plot_path_dir + "/" + file_name);
    vector<int32_t> roi = computeRoi(poses_gt, poses_result);
    plotPathPlot(plot_path_dir, roi, sequence_);

    // save + plot individual errors
    char prefix[16];
    sprintf(prefix, "%02d", sequence_);
    saveErrorPlots(seq_err, plot_error_dir, prefix);
    plotErrorPlots(plot_error_dir, prefix);

    // save + plot total errors + summary statistics
    if (total_err.size() > 0) {
        char prefix[16];
        sprintf(prefix, "avg");
        saveErrorPlots(total_err, plot_error_dir, prefix);
        plotErrorPlots(plot_error_dir, prefix);
        saveStats(total_err, std::string(results_dir));
    }

    // success
    return true;
}


int32_t main (int32_t argc,char *argv[])
{
    // we need 3 arguments
    if (argc != 3) {
        cout << "Usage: ./eval_odometry result_path kitti_sequence" << endl;
        return 1;
    }

    // read arguments
    string result_odom = argv[1];

    // init notification mail
    Mail *mail;
    mail = new Mail();
    mail->msg("Thank you for participating in our evaluation!");

    // run evaluation
    bool success = eval2(result_odom, atoi(argv[2]), mail);

    if (!success)
        cerr << "evaluate odometry failed!" << endl;


    // send mail and exit
    delete mail;

    return 0;
}

