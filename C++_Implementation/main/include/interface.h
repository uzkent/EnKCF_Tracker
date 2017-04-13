/// Copyright (c) 2016 AutelRobotics. All Rights Reserved
///
/// \edge_boxes_interface.h
///
/// \Definition : This is interface function of edge box.
///
///  \author XipengCui
///  \Date : Created on: Jan 1, 2017
///
#ifndef _INTERFACE_H_
#define _INTERFACE_H_

/// 
/// Initialization box only one time
/// 
/// \param[in] model_file: model path, such as: /home/autel/edgebox/model_train.txt
///
void InitializedBox(char* model_file);

#endif
