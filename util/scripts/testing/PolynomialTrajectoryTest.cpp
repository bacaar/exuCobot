/*
Author: Aaron Bacher
Date: 18.08.2022

Offline testing script for interpolating trajectory with c++
Class "TestClass" is pendant to MyCartesianPoseController

for comments explaining functionalities of methods and attributes see real controller
*/


#include <vector>
#include <array>

#include <iostream>
#include <fstream>


class TestClass{

public:
    TestClass(std::vector<double> tVec, std::vector<double> pVec);
    void loop();

private:
    std::vector<double> calcCoefs(double s0, double ds0, double dds0, double sT, double dsT, double ddsT, double T);
    std::vector<double> evaluatePolynom(std::vector<double> &coef, double t);
    void updateTrajectory();


private:
    const double segment_duration_ = 0.01;

    std::vector<std::vector<double>> current_state_;
    std::vector<double> current_target_;
    std::vector<std::vector<double>> position_buffer_;
    int position_buffer_index_reading_;
    int position_buffer_index_writing_;
    std::vector<std::vector<double>> coefs_;

    int position_buffer_length_;
    std::vector<double>tVec_;

};

TestClass::TestClass(std::vector<double> tVec, std::vector<double> pVec){

    current_state_ = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
    current_target_= std::vector<double>(3, 0);
    //position_buffer_ = std::vector<std::vector<double>>(position_buffer_length_, std::vector<double>(3, 0));
    position_buffer_index_reading_ = 0;
    position_buffer_index_writing_ = 1;
    coefs_ = std::vector<std::vector<double>>(3, std::vector<double>(6, 0));

    position_buffer_length_ = tVec.size();
    tVec_ = tVec;

    for(int i = 0; i < pVec.size(); ++i){
        position_buffer_.push_back({0, pVec.at(i), 0});
    }
}

std::vector<double> TestClass::calcCoefs(double s0, double ds0, double dds0, double sT, double dsT, double ddsT, double T){
        double T2 = T*T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        std::vector<double> solution(6, 0);

        solution[0] = -dds0/(2*T3) + ddsT/(2*T3) - 3*ds0/T4 - 3*dsT/T4 - 6*s0/T5 + 6*sT/T5;
        solution[1] = 3*dds0/(2*T2) - ddsT/T2 + 8*ds0/T3 + 7*dsT/T3 + 15*s0/T4 - 15*sT/T4;
        solution[2] = -3*dds0/(2*T) + ddsT/(2*T) - 6*ds0/T2 - 4*dsT/T2 - 10*s0/T3 + 10*sT/T3;
        solution[3] = dds0/2;
        solution[4] = ds0;
        solution[5] = s0;

        return solution;
    }

std::vector<double> TestClass::evaluatePolynom(std::vector<double> &coef, double t){

    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    std::vector<double> stateVec(3, 0);

    stateVec[0] =    coef[0]*t5 +    coef[1]*t4 +   coef[2]*t3 +   coef[3]*t2 + coef[4]*t + coef[5];
    stateVec[1] =  5*coef[0]*t4 +  4*coef[1]*t3 + 3*coef[2]*t2 + 2*coef[3]*t  + coef[4];
    stateVec[2] = 20*coef[0]*t3 + 12*coef[1]*t2 + 6*coef[2]*t  + 2*coef[3];

    return stateVec;

}

void TestClass::updateTrajectory(){

        // get current pose
        //std::array<double, 16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;

        // index of next positions
        int i1 = (position_buffer_index_reading_ + 1) % position_buffer_length_; // next position
        int i2 = (position_buffer_index_reading_ + 2) % position_buffer_length_; // second next position

        // calculate desired velocity for end of (first) segment as mean velocity of next two segments
        std::array<double, 3> next_velocity{};
        
        /*
        next_velocity[0] = (position_buffer_[i2][0] - current_pose[12])/(segment_duration_*2);
        next_velocity[1] = (position_buffer_[i2][1] - current_pose[13])/(segment_duration_*2);
        next_velocity[2] = (position_buffer_[i2][2] - current_pose[14])/(segment_duration_*2);
        */

        next_velocity[0] = (position_buffer_[i2][0] - current_state_[0][0])/(segment_duration_*2);
        next_velocity[1] = (position_buffer_[i2][1] - current_state_[1][0])/(segment_duration_*2);
        next_velocity[2] = (position_buffer_[i2][2] - current_state_[2][0])/(segment_duration_*2);
        

        // calculate polynom coefficients
        // TODO: using current velocity and acceleration from current_state_ vector is not 100% correct, as they are the vel and acc from last step
        /*
        coefs_[0] = calcCoefs(current_pose[12], current_state_[0][1], current_state_[0][2], position_buffer_[i1][0], next_velocity[0], 0, segment_duration_);
        coefs_[1] = calcCoefs(current_pose[13], current_state_[1][1], current_state_[1][2], position_buffer_[i1][1], next_velocity[1], 0, segment_duration_);
        coefs_[2] = calcCoefs(current_pose[14], current_state_[2][1], current_state_[2][2], position_buffer_[i1][2], next_velocity[2], 0, segment_duration_);
        */


        //current_state_[1][2] = 0;
        
        coefs_[0] = calcCoefs(current_state_[0][0], current_state_[0][1], current_state_[0][2], position_buffer_[i1][0], next_velocity[0], 0, segment_duration_);
        coefs_[1] = calcCoefs(current_state_[1][0], current_state_[1][1], current_state_[1][2], position_buffer_[i1][1], next_velocity[1], 0, segment_duration_);
        coefs_[2] = calcCoefs(current_state_[2][0], current_state_[2][1], current_state_[2][2], position_buffer_[i1][2], next_velocity[2], 0, segment_duration_);
        

        // for next segment
        int old = position_buffer_index_reading_;
        position_buffer_index_reading_ = (position_buffer_index_reading_ + 1) % position_buffer_length_;
    }

void TestClass::loop(){

    double t = 0;
    double segment_time_ = 0;
    double dt = 0.001;

    std::ofstream interpolatedFile;
    interpolatedFile.open("interpolated.dat", std::ios::out );
    std::ofstream targetFile;
    targetFile.open("target.dat", std::ios::out);

    int j = 0;
    while(t < tVec_.at(tVec_.size()-1)){

        // if segment_duration_ has passed, calc new trajectory
            if(segment_time_ >= segment_duration_){
                
                // it should be only minimal above segment_duration
                current_state_[0] = evaluatePolynom(coefs_[0], segment_time_);
                current_state_[1] = evaluatePolynom(coefs_[1], segment_time_);
                current_state_[2] = evaluatePolynom(coefs_[2], segment_time_);
                
                updateTrajectory();
                if (position_buffer_index_reading_ != 0)    // only for last iteration as index jumps to 0 because of modulo
                    targetFile << "[" << t << ", " << position_buffer_[position_buffer_index_reading_-1][1] << "]," << std::endl;

                // reset segment_time_ as new one starts now
                // normally subtracting duration once should be enough
                while(segment_time_ > segment_duration_){
                    segment_time_ -= segment_duration_;
                }
            }

            // if within semgent_duration, calc new state
            if(segment_time_ <= segment_duration_){
                // calculat new positions, velocities and accelerations
                current_state_[0] = evaluatePolynom(coefs_[0], segment_time_);
                current_state_[1] = evaluatePolynom(coefs_[1], segment_time_);
                current_state_[2] = evaluatePolynom(coefs_[2], segment_time_);

                // results to outputfile (interpolated trajectory)
                interpolatedFile << "[" << t << ", " << current_state_[1][0] << ", " << current_state_[1][1] << ", " << current_state_[1][2] << "]," << std::endl;

                // compose new pose
                //std::array<double, 16> new_pose = current_pose;
                //new_pose[12] = current_state_[0][0];
                //new_pose[13] = current_state_[1][0];
                //new_pose[14] = current_state_[2][0];

                // pass new pose to robot control
                //cartesian_pose_handle_->setCommand(new_pose);
                //newStatePublished = true;
            }
            else{
                std::cerr << "ERROR: semgment_time > segment_duration\n";
                exit(-1);
            }            

        segment_time_ += dt;
        t += dt;
        j++;
    }

    interpolatedFile.close();
    targetFile.close();
}

int main(){

    std::cout << "Hello world :)" << std::endl;

    std::vector<double> tvec = {
        4.889748000000000011e-02,
        4.901780699999999658e-02,
        6.349265900000000684e-02,
        6.973407800000000489e-02,
        7.909960200000000496e-02,
        8.850882200000000088e-02,
        1.027696250000000033e-01,
        1.487194330000000120e-01,
        1.585193340000000117e-01,
        1.711780599999999930e-01,
        1.711939349999999915e-01,
        1.739765149999999982e-01,
        1.740236679999999925e-01,
        1.740352559999999993e-01,
        1.788149270000000124e-01,
        1.825795889999999866e-01,
        1.923946960000000039e-01,
        2.086995490000000120e-01,
        2.181678360000000039e-01,
        2.287501630000000064e-01,
        2.387508829999999971e-01,
        2.427746869999999890e-01,
        2.589643939999999867e-01,
        2.698737290000000066e-01,
        2.782617920000000078e-01,
        2.982044369999999889e-01,
        3.087120990000000176e-01,
        3.087326570000000214e-01,
        3.125126080000000250e-01,
        3.287897790000000042e-01,
        3.327807199999999743e-01,
        3.591906040000000244e-01,
        3.729045329999999825e-01,
        3.729297200000000201e-01,
        3.783726809999999885e-01,
        3.885856429999999806e-01,
        3.922219949999999900e-01,
        4.023827420000000155e-01,
        4.178160249999999798e-01,
        4.223829439999999824e-01,
        4.382624419999999743e-01,
        4.428402280000000024e-01,
        4.622359639999999992e-01,
        4.623306409999999866e-01,
        4.724262710000000087e-01,
        5.182461630000000374e-01,
        5.183141220000000438e-01,
        5.385621980000000475e-01,
        5.385787800000000347e-01,
        5.385853259999999754e-01,
        5.486531500000000339e-01,
        5.488181720000000485e-01,
        5.590011200000000180e-01,
        5.693545570000000389e-01,
        5.794788849999999991e-01,
        6.002820979999999862e-01,
        6.084855129999999779e-01,
        6.085837859999999599e-01,
        6.178437740000000122e-01,
        6.282681860000000063e-01,
        6.328993800000000114e-01,
        6.484622949999999664e-01,
        6.586783480000000246e-01,
        6.891328300000000295e-01,
        6.891678980000000010e-01,
        6.892187290000000299e-01,
        6.991992289999999777e-01,
        7.090370839999999841e-01,
        7.193248819999999988e-01,
        7.380660749999999881e-01,
        7.380765270000000378e-01,
        7.484566299999999561e-01,
        7.689244560000000339e-01,
        7.778439419999999549e-01,
        7.778584339999999875e-01,
        7.885223930000000436e-01,
        7.922950840000000383e-01,
        8.032805389999999601e-01,
        8.298293719999999540e-01,
        8.298792780000000535e-01,
        8.397468479999999902e-01,
        8.492440509999999998e-01,
        8.688370119999999641e-01,
        8.688709510000000025e-01,
        8.733657179999999576e-01,
        8.828345389999999737e-01,
        8.937510309999999736e-01,
        9.033915340000000516e-01,
        9.137243819999999461e-01,
        9.291718740000000087e-01,
        9.591547350000000360e-01,
        9.591991490000000287e-01,
        9.698288919999999980e-01,
        9.698618359999999772e-01,
        9.736361310000000158e-01,
        9.830374320000000443e-01,
        9.928072719999999629e-01,
        1.009767622999999892e+00,
        1.029749264000000109e+00,
        1.029778011000000104e+00,
        1.049043844999999919e+00,
        1.049094275999999937e+00,
        1.059171681000000032e+00,
        1.073778549000000027e+00,
        1.079797192000000017e+00,
        1.082804730999999965e+00,
        1.093325520999999911e+00,
        1.108311395999999949e+00,
        1.118823734000000014e+00,
        1.140557833999999993e+00,
        1.140687977000000020e+00,
        1.149209555000000105e+00,
        1.160014319000000071e+00,
        1.168033107000000070e+00,
        1.183488958999999952e+00,
        1.183567671999999904e+00,
        1.199385248000000015e+00,
        1.210186017000000058e+00,
        1.213740915000000031e+00,
        1.229300322000000056e+00,
        1.233883665000000018e+00,
        1.258904063999999989e+00,
        1.259280956000000007e+00,
        1.269842606000000096e+00,
        1.273241931999999910e+00,
        1.293897625000000051e+00,
        1.299446687999999961e+00,
        1.309193920999999872e+00,
        1.313624815999999917e+00,
        1.328178403999999979e+00,
        1.339096494999999942e+00,
        1.354188632999999919e+00,
        1.354241411999999922e+00,
        1.390258117000000126e+00,
        1.409042450000000057e+00,
        1.409080777000000007e+00,
        1.419409235999999908e+00,
        1.423863458000000026e+00,
        1.423965619999999932e+00,
        1.423994128999999997e+00,
        1.440272476000000079e+00,
        1.443443183000000074e+00,
        1.459896287999999931e+00,
        1.469598600999999949e+00,
        1.472763319999999876e+00,
        1.489205088000000066e+00,
        1.523465880000000050e+00,
        1.529489116000000148e+00,
        1.529588461999999982e+00,
        1.548904081999999960e+00,
        1.553293626999999955e+00,
        1.553329747000000038e+00,
        1.553412052999999959e+00,
        1.568344783999999992e+00,
        1.578629394000000019e+00,
        1.587558044999999973e+00,
        1.618190192000000138e+00,
        1.618196423999999967e+00,
        1.618206339999999965e+00,
        1.628417341000000018e+00,
        1.648040594000000025e+00,
        1.652776742999999993e+00,
        1.658341602999999997e+00,
        1.668397773000000139e+00,
        1.672883813999999969e+00,
        1.688525928000000009e+00,
        1.698706411999999943e+00,
        1.712565748999999915e+00,
        1.712570667000000046e+00,
        1.722692113999999997e+00,
        1.738547744999999978e+00,
        1.758052514000000066e+00,
        1.758059481000000090e+00,
        1.808124739000000147e+00,
        1.808135140999999946e+00,
        1.808141291999999956e+00,
        1.808183231999999974e+00,
        1.812341994000000067e+00,
        1.812351172000000066e+00,
        1.852918837000000041e+00,
        1.858430355999999950e+00,
        1.858506210999999908e+00,
        1.868601272000000035e+00,
        1.868610148000000137e+00,
        1.877759956999999869e+00,
        1.888085283000000114e+00,
        1.898337428000000049e+00,
        1.908639882000000121e+00,
        1.917842825000000140e+00,
        1.938332283000000045e+00,
        1.938336704999999993e+00,
        1.988301948000000152e+00,
        2.008418138000000130e+00,
        2.008422061000000092e+00,
        2.008434707000000152e+00,
        2.008437810999999851e+00,
        2.008440578000000087e+00,
        2.018448880000000223e+00,
        2.018455399000000039e+00,
        2.028569105999999955e+00
    };
    std::vector<double> pvec = {
        4.769210092045206295e-02,
        4.595609288870805553e-02,
        4.415239596025011259e-02,
        4.228365097109687554e-02,
        4.035259600196117180e-02,
        3.836206248397966423e-02,
        3.631497114442761021e-02,
        3.421432779692457071e-02,
        3.206321898118513758e-02,
        2.986480745792929881e-02,
        2.762232756512017495e-02,
        2.533908044225485767e-02,
        2.301842912998552926e-02,
        2.066379355287450981e-02,
        1.827864539360979368e-02,
        1.586650286749158312e-02,
        1.343092540646395605e-02,
        1.097550826239124255e-02,
        8.503877039663265691e-03,
        6.019682167561879460e-03,
        3.526593323110560618e-03,
        1.028293815380831866e-03,
        -1.471525057578748630e-03,
        -3.969169667893623910e-03,
        -6.460949721702746729e-03,
        -8.943183916286345791e-03,
        -1.141220557910505029e-02,
        -1.386436827746617695e-02,
        -1.629605138768974726e-02,
        -1.870366561283165474e-02,
        -2.108365843830117825e-02,
        -2.343251951498670493e-02,
        -2.574678595988144636e-02,
        -2.802304756457552060e-02,
        -3.025795190242308585e-02,
        -3.244820932565573024e-02,
        -3.459059784420825956e-02,
        -3.668196787854616225e-02,
        -3.871924687931116438e-02,
        -4.069944380716217225e-02,
        -4.261965346673690647e-02,
        -4.447706068922352429e-02,
        -4.626894435858441845e-02,
        -4.799268127702194242e-02,
        -4.964574986580649885e-02,
        -5.122573369810257127e-02,
        -5.273032486092321669e-02,
        -5.415732714379939416e-02,
        -5.550465905219459373e-02,
        -5.677035664409446625e-02,
        -5.795257618855753634e-02,
        -5.904959664535813779e-02,
        -6.005982196513071081e-02,
        -6.098178320967595489e-02,
        -6.181414049230582464e-02,
        -6.255568473827266551e-02,
        -6.320533926545768288e-02,
        -6.376216118559985269e-02,
        -6.422534262640211544e-02,
        -6.459421177487856269e-02,
        -6.486823374232475459e-02,
        -6.504701125124667804e-02,
        -6.513028514455787565e-02,
        -6.511793471727855831e-02,
        -6.500997787090523339e-02,
        -6.480657109051879416e-02,
        -6.450800924463062636e-02,
        -6.411472520765792993e-02,
        -6.362728930484973233e-02,
        -6.304640857940468912e-02,
        -6.237292588146825523e-02,
        -6.160781877865129097e-02,
        -6.075219828769617969e-02,
        -5.980730742693074475e-02,
        -5.877451958917845332e-02,
        -5.765533673487799327e-02,
        -5.645138740525479371e-02,
        -5.516442455554848578e-02,
        -5.379632320846927662e-02,
        -5.234907792827936390e-02,
        -5.082480011616308246e-02,
        -4.922571512784013059e-02,
        -4.755415921472083696e-02,
        -4.581257629027679634e-02,
        -4.400351452370521166e-02,
        -4.212962276342002710e-02,
        -4.019364679335812873e-02,
        -3.819842542560381027e-02,
        -3.614688643333829887e-02,
        -3.404204232867424906e-02,
        -3.188698599046424498e-02,
        -2.968488614775122159e-02,
        -2.743898272506584135e-02,
        -2.515258205635007016e-02,
        -2.282905197482065773e-02,
        -2.047181678662290949e-02,
        -1.808435213663739383e-02,
        -1.567017977529649997e-02,
        -1.323286223571118470e-02,
        -1.077599743084745043e-02,
        -8.303213180864243981e-03,
        -5.818161681067746116e-03,
        -3.324513921231897484e-03,
        -8.259540672697074370e-04,
        1.673826183560933600e-03,
        4.171133262610093873e-03,
        6.662277393558957428e-03,
        9.143578248504202755e-03,
        1.161137058503680652e-02,
        1.406200985304384421e-02,
        1.649187776011151740e-02,
        1.889738778461713675e-02,
        2.127499062585780365e-02,
        2.362117958086751290e-02,
        2.593249583793744151e-02,
        2.820553367724132343e-02,
        3.043694556941300977e-02,
        3.262344716338483330e-02,
        3.476182215529444175e-02,
        3.684892703079079013e-02,
        3.888169567360322265e-02,
        4.085714383379179804e-02,
        4.277237344965278965e-02,
        4.462457681781195618e-02,
        4.641104060659251296e-02,
        4.812914970829085259e-02,
        4.977639092652486053e-02,
        5.135035649533004065e-02,
        5.284874742716894058e-02,
        5.426937668748421650e-02,
        5.561017219385355759e-02,
        5.686917963820614652e-02,
        5.804456513092293157e-02,
        5.913461766596694869e-02,
        6.013775140647970030e-02,
        6.105250779052417975e-02,
        6.187755745686129849e-02,
        6.261170199082277676e-02,
        6.325387549046157520e-02,
        6.380314595326930238e-02,
        6.425871648379688494e-02,
        6.461992632255308955e-02,
        6.488625169654693714e-02,
        6.505730649182250858e-02,
        6.513284274827912945e-02,
        6.511275097701318959e-02,
        6.499706030033558513e-02,
        6.478593841453539337e-02,
        6.447969137537201689e-02,
        6.407876320618677290e-02,
        6.358373532844785458e-02,
        6.299532581446198876e-02,
        6.231438846193826375e-02,
        6.154191169004274986e-02,
        6.067901725657376399e-02,
        5.972695879589140500e-02,
        5.868712017728594521e-02,
        5.756101368353183645e-02,
        5.635027800949521382e-02,
        5.505667608079856112e-02,
        5.368209269273982454e-02,
        5.222853196988030966e-02,
        5.069811464698659176e-02,
        4.909307517230621087e-02,
        4.741575863450975259e-02,
        4.566861751499562416e-02,
        4.385420826768327007e-02,
        4.197518772885000526e-02,
        4.003430936005458740e-02,
        3.803441932767748224e-02,
        3.597845242314090530e-02,
        3.386942782839996724e-02,
        3.171044473184969625e-02,
        2.950467780035348486e-02,
        2.725537251364806846e-02,
        2.496584036794502470e-02,
        2.263945395608812916e-02,
        2.027964193216036382e-02,
        1.788988386894330151e-02,
        1.547370501711964330e-02,
        1.303467097556265664e-02,
        1.057638228247759216e-02,
        8.102468937539963889e-03,
        5.616584865510421132e-03,
        3.122402332094820210e-03,
        6.236063230571353699e-04,
        -1.876111102224742311e-03,
        -4.373056471442149373e-03,
        -6.863540562882963769e-03,
        -9.343884060399654246e-03,
        -1.181042318845682004e-02,
        -1.425951531599078947e-02,
        -1.668754451798259986e-02,
        -1.909092708384529935e-02,
        -2.146611696199851593e-02,
        -2.380961113031032639e-02,
        -2.611795488244927910e-02,
        -2.838774702058310950e-02,
        -3.061564494531099356e-02,
        -3.279836963417115392e-02
    };

    if(tvec.size() != pvec.size()){
        return -1;
    }
    std::cout <<tvec.size() << "\t" << pvec.size() << std::endl;

    TestClass test(tvec, pvec);
    test.loop();

    return 0;
}