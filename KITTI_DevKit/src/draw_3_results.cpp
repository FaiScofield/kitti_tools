#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <limits>

#include "mail.h"
#include "matrix.h"

using namespace std;


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
    fprintf(fp, "plot \"%02d.txt\" using 1:2 with lines lt 7 lw 2 title 'Ground Truth',", sequence);
    fprintf(fp, "\"%02d.txt\" using 3:4 with points lc rgb \"#00FF00\" pt 7 ps 0.4 lw 1 title 'Without Loop Closure',", sequence);
    fprintf(fp, "\"%02d.txt\" using 5:6 with linespoints lc rgb \"#0000FF\" pt 7 lw 1 ps 0.5 title 'With Loop Closure',", sequence);
    fprintf(fp, "\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n", sequence);

    // close file
    fclose(fp);

    // run gnuplot => create png + eps
    sprintf(command, "gnuplot %s", full_name.c_str());
    system(command);
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


    char filename[256];
    sprintf(filename, "./%02d.txt", sequence);
    savePathPlot(posesGt, posesEs, posesEs2, filename);
    vector<int32_t> roi = computeRoi(posesGt, posesEs);
    char gpfile[256];
    sprintf(gpfile, "./%02d.gp", sequence);
    plotPathPlot(gpfile, roi, sequence);

    return 0;
}
