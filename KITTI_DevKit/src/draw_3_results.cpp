#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>

#include "mail.h"
#include "matrix.h"

using namespace std;

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


vector<Matrix> loadPoses(string filename) {
    cout << "Reading poses from " << filename << endl;

    vector<Matrix> poses;
    FILE *fp = fopen(filename.c_str(), "r");
    if (!fp) {
        cerr << "compare file can not be found!" << endl;
        return poses;
    }

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

void saveSequenceErrors (vector<errors> &err1, vector<errors> &err2, string file_name) {
    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(), "w");

    // write to file
//    for (vector<errors>::iterator it=err1.begin(); it!=err1.end(); it++) {
//        fprintf(fp, "%d %d %f %f %f %f\n", it->first_frame, it->last_frame, it->r_err, it->t_err, it->len, it->speed);

//    }
    size_t n1 = err1.size(), n2 = err2.size();
    if (n1 != n2) {
        cerr << "Wrong size of two error sequences!." << endl;
        return;
    }
    for (size_t i=0; i<n1; ++i) {
        if (err1[i].first_frame != err2[i].first_frame ||
            err1[i].last_frame != err2[i].last_frame ||
            err1[i].len != err2[i].len ||
            err1[i].speed != err2[i].speed) {
            cerr << "Warning: error data not equal!\n";
            continue;
        }
        fprintf(fp, "%d %d %f %f %f %f %f %f\n", err1[i].first_frame, err1[i].last_frame, err1[i].r_err, err1[i].t_err, err2[i].r_err, err2[i].t_err, err1[i].len, err1[i].speed);
    }

    // close file
    fclose(fp);
}

void savePathPlot (vector<Matrix> &poses_gt, vector<Matrix> &poses1, vector<Matrix> &poses2, string file_name) {
    // parameters
    int32_t step_size = 5;

    // open file
    FILE *fp = fopen(file_name.c_str(), "w");

    // save x/z coordinates of all frames to file
    for (int32_t i=0; i<poses_gt.size(); i+=step_size)
    fprintf(fp, "%f %f %f %f %f %f\n",
            poses_gt[i].val[0][3], poses_gt[i].val[2][3],
            poses1[i].val[0][3], poses1[i].val[2][3],
            poses2[i].val[0][3], poses2[i].val[2][3]);

    // close file
    fclose(fp);
}


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

void plotPathPlot (string full_name, vector<int32_t> &roi, int32_t sequence) {
    // gnuplot file name
    char command[1024];
//    char file_name[256];
//    sprintf(file_name, "%02d.gp", sequence);
//    string full_name = dir + "/" + file_name;

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
    fprintf(fp, "plot \"%02d_path.txt\" using 1:2 with lines lt 7 lw 2 title 'Ground Truth',", sequence);
    fprintf(fp, "\"%02d_path.txt\" using 3:4 with points lc rgb \"#00FF00\" pt 7 ps 0.4 lw 1 title 'Only L-SLAM',", sequence);
    fprintf(fp, "\"%02d_path.txt\" using 5:6 with linespoints lc rgb \"#0000FF\" pt 7 lw 1 ps 0.5 title 'Full System',", sequence);
    fprintf(fp, "\"< head -1 %02d_path.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n", sequence);

    // close file
    fclose(fp);

    // run gnuplot => create png + eps
    sprintf(command, "gnuplot %s", full_name.c_str());
    system(command);
}


void saveErrorPlots(vector<errors> &seqErr1, vector<errors>& seqErr2, char* prefix) {
    // file names
    char file_name_tl[1024]; sprintf(file_name_tl, "./%s_tl.txt", prefix);
    char file_name_rl[1024]; sprintf(file_name_rl, "./%s_rl.txt", prefix);
    char file_name_ts[1024]; sprintf(file_name_ts, "./%s_ts.txt", prefix);
    char file_name_rs[1024]; sprintf(file_name_rs, "./%s_rs.txt", prefix);

    // open files
    FILE *fp_tl = fopen(file_name_tl, "w");
    FILE *fp_rl = fopen(file_name_rl, "w");
    FILE *fp_ts = fopen(file_name_ts, "w");
    FILE *fp_rs = fopen(file_name_rs, "w");

    // for each segment length do
    for (int32_t j=0; j<num_lengths; j++) {
        float t_err1 = 0, t_err2 = 0;
        float r_err1 = 0, r_err2 = 0;
        float num   = 0;

        // for all errors do
//        for (vector<errors>::iterator it=seqErr1.begin(); it!=seqErr1.end(); it++) {
//            if (fabs(it->len - lengths[i]) < 1.0) { // 用<1.0代替==
//                t_err1 += it->t_err;
//                r_err += it->r_err;
//                num++;
//            }
//        }

        for (size_t i=0; i<seqErr1.size(); ++i) {
            if (fabs(seqErr1[i].len - lengths[j]) < 1.0) {
                t_err1 += seqErr1[i].t_err;
                r_err1 += seqErr1[i].r_err;
                t_err2 += seqErr2[i].t_err;
                r_err2 += seqErr2[i].r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_tl, "%f %f %f\n", lengths[j], t_err1/num, t_err2/num);
            fprintf(fp_rl, "%f %f %f\n", lengths[j], r_err1/num, r_err2/num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed=2; speed<25; speed+=2) {
        float t_err1 = 0, t_err2 = 0;
        float r_err1 = 0, r_err2 = 0;
        float num   = 0;

        // for all errors do
//        for (vector<errors>::iterator it=seqErr1.begin(); it!=seqErr1.end(); it++) {
//            if (fabs(it->speed - speed) < 2.0) {
//                t_err1 += it->t_err;
//                r_err1 += it->r_err;
//                num++;
//            }
//        }
        for (size_t i=0; i<seqErr1.size(); ++i) {
            if (fabs(seqErr1[i].speed - speed) < 2.0) {
                t_err1 += seqErr1[i].t_err;
                r_err1 += seqErr1[i].r_err;
                t_err2 += seqErr2[i].t_err;
                r_err2 += seqErr2[i].r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_ts, "%f %f %f\n", speed, t_err1/num, t_err2/num);
            fprintf(fp_rs, "%f %f %f\n", speed, r_err1/num, r_err2/num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void plotErrorPlots (char* prefix) {
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
        sprintf(full_name, "./%s", file_name);

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
            case 0: fprintf(fp,"1:($2*100) title 'Only L-SLAM' lc rgb \"#00FF00\" pt 4 w linespoints, \"%s_%s.txt\" using 1:($3*100) title 'Full System' lc rgb \"#0000FF\" pt 5 w linespoints\n",prefix,suffix); break;
            case 1: fprintf(fp,"1:($2*57.3) title 'Only L-SLAM' lc rgb \"#00FF00\" pt 4 w linespoints, \"%s_%s.txt\" using 1:($3*57.3) title 'Full System' lc rgb \"#0000FF\" pt 5 w linespoints\n",prefix,suffix); break;
            case 2: fprintf(fp,"($1*3.6):($2*100) title 'Only L-SLAM' lc rgb \"#00FF00\" pt 4 w linespoints, \"%s_%s.txt\" using ($1*3.6):($3*100) title 'Full System' lc rgb \"#0000FF\" pt 5 w linespoints\n",prefix,suffix); break;
            case 3: fprintf(fp,"($1*3.6):($2*57.3) title 'Only L-SLAM' lc rgb \"#00FF00\" pt 4 w linespoints, \"%s_%s.txt\" using ($1*3.6):($3*57.3) title 'Full System' lc rgb \"#0000FF\" pt 5 w linespoints\n",prefix,suffix); break;
        }
//        fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(command,"gnuplot %s; ", file_name);
        system(command);
    }

//    sprintf(command,"rm *.gp *.txt");
//    system(command);
}


int main (int argc,char *argv[])
{
    // we need 3 arguments
    if (argc != 4) {
        cout << "Usage: ./draw_results result_path1 result_path2 kitti_sequence" << endl;
        return -1;
    }

    // read arguments
    string estimateOdomFile = argv[1];
    string estimateOdomFile2 = argv[2];
    int sequence = atoi(argv[3]);
    char groundTruthFile[256];
    sprintf(groundTruthFile, "%02d.txt", sequence);
    vector<Matrix> posesEs = loadPoses(estimateOdomFile);
    vector<Matrix> posesEs2 = loadPoses(estimateOdomFile2);
    vector<Matrix> posesGt = loadPoses("../poses/" + string(groundTruthFile));

    // check for errors
    if (posesEs.size()==0) {
        cerr << "Couldn't read (all) es poses." << endl;
        return -1;
    }
    if (posesEs2.size()==0) {
        cerr << "Couldn't read (all) es2 poses." << endl;
        return -1;
    }
    if (posesGt.size()==0) {
        cerr << "Couldn't read (all) gt poses." << endl;
        return -1;
    }

    // unequal size
    int sizeEs1 = posesEs.size(), sizeEs2 = posesEs2.size(), sizeGt = posesGt.size();
    int minSize = min(min(sizeEs1, sizeEs2), sizeGt);
    printf("min size of all three poses: %d\n", minSize);
    if (sizeEs1 > minSize)
        posesEs.erase(posesEs.begin()+minSize, posesEs.end());
    if (sizeEs2 > minSize)
        posesEs2.erase(posesEs2.begin()+minSize, posesEs2.end());
    if (sizeGt > minSize)
        posesGt.erase(posesGt.begin()+minSize, posesGt.end());


    if (posesEs.size() == posesEs2.size() && posesEs2.size() == posesGt.size())
        printf("Same size of all three poses. Continue.\n");
    else {
        printf("Not same size of all three poses. Return.\n");
        return -1;
    }


    char pathFile[256];
    sprintf(pathFile, "./%02d_path.txt", sequence);
    savePathPlot(posesGt, posesEs, posesEs2, pathFile);
    vector<int32_t> roi = computeRoi(posesGt, posesEs);
    char gpPath[256];
    sprintf(gpPath, "./%02d_path.gp", sequence);
    plotPathPlot(gpPath, roi, sequence);

    // compute sequence errors
    vector<errors> seqErr1 = calcSequenceErrors(posesGt, posesEs);
    vector<errors> seqErr2 = calcSequenceErrors(posesGt, posesEs2);
    char errorFile[256];
    sprintf(errorFile, "./%02d_error.txt", sequence);
    saveSequenceErrors(seqErr1, seqErr2, errorFile);

    char prefix[16];
    sprintf(prefix, "%02d", sequence);
    saveErrorPlots(seqErr1, seqErr2, prefix);
    plotErrorPlots(prefix);



    return 0;
}
