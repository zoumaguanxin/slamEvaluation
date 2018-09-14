/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "evaluation.h"
Eigen::Vector2f squareandCut(const Eigen::Vector3f& x)
{
  Eigen::Vector2f v;
  v(0)=sqrt(x(0)*x(0)+x(1)*x(1)); 
  v(1)=x(2);
  return v;
 }

 Eigen::MatrixXf sum(const Eigen::MatrixXf& x,const Eigen::MatrixXf & y)
 {
   return x+y;
}
 
 
Eigen::Vector2f evaluation::computeMSE()
{
  std::vector<Eigen::Vector2f> tem;
  std::vector<Eigen::Vector2f>::iterator it=tem.begin();
  //transform(v_pose.begin(),v_pose.end(), tem.begin(),squareandCut);
  for(std::vector<Eigen::Vector3f>::iterator it1=v_pose.begin();it1!=v_pose.end();it1++)
  {
    /*
    *it=squareandCut(*it1);      
    ++it;
    */
    tem.push_back(squareandCut(*it1));
  }
  Eigen::Vector2f MSE;
  MSE=accumulate(tem.begin(),tem.end(),MSE,sum);
  MSE=MSE/v_pose.size();
  return MSE;
}

Eigen::Vector3f evaluation::computeMAE()
{
  Eigen::Matrix<float,3,1> tem;
  tem.setZero();
  tem=accumulate(v_pose.begin(),v_pose.end(),tem,sum);
   tem=tem/v_pose.size();
   return tem;
}
