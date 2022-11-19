 
#include <iostream>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <assert.h>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
 
using namespace std;
using namespace Eigen;
//using namespace ceres;
 
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;
 
typedef struct {
    int id = 0;
    double param[7] = {0};
    double se3[6] = {0};//(t,r)
} param_type;
 

Matrix6d JRInv(const Sophus::SE3d &e){
    Matrix6d J;
    J.block(0,0,3,3) = Sophus::SO3d::hat(e.so3().log());
    J.block(0,3,3,3) = Sophus::SO3d::hat(e.translation());
    J.block(3,0,3,3) = Matrix3d::Zero(3,3);
    J.block(3,3,3,3) = Sophus::SO3d::hat(e.so3().log());
    J = 0.5 * J + Matrix6d::Identity();
    return J;
//    return Matrix6d::Identity();
}
 
class Parameterization:public ceres::LocalParameterization
{
public:
    Parameterization(){}
    virtual ~Parameterization(){}
    virtual bool Plus(const double* x,
                      const double * delta,
                      double *x_plus_delta)const
    {
        Vector6d lie(x);
        Vector6d lie_delta(delta);
        Sophus::SE3d T=Sophus::SE3d::exp(lie);
        Sophus::SE3d T_delta=Sophus::SE3d::exp(lie_delta);
 
        Eigen::Map<Vector6d> x_plus(x_plus_delta);
        x_plus=(T_delta*T).log();
        return true;
 
    }
    //流形到其切平面的雅克比矩阵
    virtual bool ComputeJacobian(const double *x,
                                 double * jacobian) const
    {
        ceres::MatrixRef(jacobian,6,6)=ceres::Matrix::Identity(6,6);//ceres::MatrixRef()函数也类似于Eigen::Map,通过模板参数传入C++指针将c++数据映射为Ceres::Matrix
        return true;
    }
    //定义流形和切平面维度：在本问题中是李代数到李代数故都为6
    virtual int GlobalSize()const{return 6;}
    virtual int LocalSize()const {return 6;}
};
 
class PoseGraphCFunc : public ceres::SizedCostFunction<6,6,6>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PoseGraphCFunc(const Sophus::SE3d measure,const Matrix6d covariance):_measure(measure),_covariance(covariance){}
    ~PoseGraphCFunc(){}
    virtual bool Evaluate(double const* const* parameters,
                          double *residuals,
                          double **jacobians)const
    {
        Sophus::SE3d pose_i=Sophus::SE3d::exp(Vector6d(parameters[0]));//Eigen::MtatrixXd可以通过double指针来构造
        Sophus::SE3d pose_j=Sophus::SE3d::exp(Vector6d(parameters[1]));
        Eigen::Map<Vector6d> residual(residuals);//Eigen::Map<typename MatrixX>(ptr)函数的作用是通过传入指针将c++数据映射为Eigen上的Matrix数据，有点与&相似
        residual=(_measure.inverse()*pose_i.inverse()*pose_j).log();
        Matrix6d sqrt_info=Eigen::LLT<Matrix6d>(_covariance).matrixLLT().transpose();
        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> jacobian_i(jacobians[0]);
                Matrix6d Jr=JRInv(Sophus::SE3d::exp(residual));
                jacobian_i=(sqrt_info)*(-Jr)*pose_j.inverse().Adj();
            }
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double,6,6, Eigen::RowMajor>> jacobian_j(jacobians[1]);
                Matrix6d J = JRInv(Sophus::SE3d::exp(residual));
                jacobian_j = sqrt_info * J * pose_j.inverse().Adj();
            }
        }
        residual=sqrt_info*residual;
        return true;
 
    }
private:
    const Sophus::SE3d _measure;
    const Matrix6d _covariance;
 
};
 
 
 
void Convert2se3(param_type &_p){
    Quaterniond q(_p.param[6], _p.param[3], _p.param[4], _p.param[5]);
    Vector3d t(_p.param[0], _p.param[1], _p.param[2]);
    Vector6d se = Sophus::SE3d(q.normalized(),t).log();
//    auto tmp = Sophus::SE3(q,t).log();
//    auto tmp1 = Sophus::SE3::exp(tmp);
    for(int i = 0; i < 6; i++){
        _p.se3[i] = se(i, 0);
    }
}
 
 
int main(int argc, char **argv) {
 
    google::InitGoogleLogging(argv[0]);
 
    string fin_path = "../sphere.g2o";
 
    ceres::Problem problem;
    vector<param_type> param;
 
    ifstream fin;
    fin.open(fin_path);
   assert(fin.is_open());
    ceres::LocalParameterization *local_param = new Parameterization();
    while(!fin.eof()){
        string name;
        fin >> name;
        if(name == "VERTEX_SE3:QUAT"){
            param_type p;
            fin >> p.id;
            for(int i = 0; i < 7; i++) fin >> p.param[i];
            Convert2se3(p);
            param.push_back(p);
 
//            problem.AddParameterBlock(param.back().se3, 6, local_param);
        }
        else if(name == "EDGE_SE3:QUAT"){
            int vertex_i, vertex_j;
            fin >> vertex_i >> vertex_j;
            //输入观测值
            double m[7];//temporary measurement result
            for(int i = 0; i < 7; i++) fin >> m[i];
            Sophus::SE3d measurement(Quaternion<double>(m[6], m[3], m[4], m[5]).normalized(),
                                    Vector3d(m[0], m[1], m[2]));
            //输入信息矩阵
            Matrix6d covariance;
            for(int i = 0; i < 6 && fin.good(); i++){
                for(int j = i; j < 6 && fin.good(); j++){
                    fin >> covariance(i,j);
                    if(j != i) covariance(j,i) = covariance(i,j);
                }
            }
            ceres::LossFunction *loss = new ceres::HuberLoss(1.0);
            ceres::CostFunction *costfunc = new PoseGraphCFunc(measurement, covariance);//定义costfunction
            problem.AddResidualBlock(costfunc, loss, param[vertex_i].se3, param[vertex_j].se3);//为整体代价函数加入快，ceres的优点就是直接在param[vertex_i].se3上优化
            problem.SetParameterization(param[vertex_i].se3, local_param);//为待优化的两个位姿变量定义迭代递增方式
            problem.SetParameterization(param[vertex_j].se3, local_param);
        }
    }
    fin.close();
 
    cout << param.size() << endl;
    ceres::Solver::Options options;//配置优化设置
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type=ceres::LEVENBERG_MARQUARDT;
    options.max_linear_solver_iterations = 50;
    options.minimizer_progress_to_stdout = true;
 
    ceres::Solver::Summary summary;//summary用来存储优化过程
    ceres::Solve(options, &problem, &summary);//开始优化
 
    cout << summary.FullReport() << endl;
    std::ofstream txt("./result_ceres.g2o");
    for( int i=0; i < param.size(); i++ )
    {
        Eigen::Map<const Eigen::Matrix<double,6,1>> poseAVec6d( param[i].se3 );
        Sophus::SE3d poseSE3 = Sophus::SE3d::exp(poseAVec6d);
        Quaternion<double> q = poseSE3.so3().unit_quaternion();//将SE3中的SO3转换为四元数
 
        txt << "VERTEX_SE3:QUAT" << ' ';
        txt << i << ' ';
        txt << poseSE3.translation().transpose() << ' ';
        txt << q.x() <<' '<< q.y()<< ' ' << q.z() <<' '<< q.w()<<' ' << endl;
    }
    fin.open(fin_path);
    while(!fin.eof()){
        string s;
        getline(fin, s);
        if(s[0] != 'E') continue;
        else txt << s << endl;
    }
    fin.close();
    txt.close();
    return 0;
}
 