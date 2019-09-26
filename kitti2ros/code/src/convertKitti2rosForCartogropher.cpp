//
// Created by olga on 23.09.19.
//

//
//I exapcted next format of data:
//folderWithData:
//    velodynePoint
//        data
//            0000000000.ply
//            0000000001.ply
//            ...
//        timestamps.txt
//    oxts
//        data
//            0000000000.txt
//            0000000001.txt
//            ...
//        timestamps.txt
//
//if you store your data in other format please change a hardcoded names

#include <KittyToRosConverter.h>

int main(){
    string folderWithKitti = "/home/olga/data/kitti/2011_09_26/2011_09_26_drive_0005_sync";
    string outputKitti = "/home/olga/data/kitti/my_visualization/2011_09_26_drive_0005_sync_test.bag";

    KittiConverter converter(folderWithKitti, outputKitti);
    converter.convertImuAndGps();
    converter.convertPointCloud();
}

