#ifndef __COMMAND_H__
#include "Command.h"
#endif //__COMMAND_H__

#ifndef __C3DFILEINFO_H__
#include "C3dFileInfo.h"
#endif	//__C3DFILEINFO_H__

#ifndef __ARTICULATEDBODY_H__
#include "ArticulatedBody.h"
#endif	//__ARTICULATEDBODY_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#include "My_Math_Lib.h"

using namespace std;

int readSkelFile( FILE* file, ArticulatedBody* skel );

extern RealTimeIKUI *UI;

void LoadModel(void *v)
{
  char *params = (char*)v;
  if(!params){
    params = (char*)fl_file_chooser("Open File?", "{*.skel}", "../src/skels" );
  }

  if(!params)
    return;

  FILE *file = fopen(params, "r");
    
  if(file == NULL){
    cout << "Skel file does not exist" << endl;
    return;
  }

  ArticulatedBody *mod = new ArticulatedBody();
  UI->mData->mModels.push_back(mod);
  UI->mData->mSelectedModel = mod;

  readSkelFile(file, mod);
  UI->CreateDofSliderWindow();

  mod->InitModel();
  UI->mGLWindow->mShowModel = true;
  UI->mShowModel_but->value(1);
  UI->mGLWindow->refresh();
  
  cout << "number of dofs in model: " << UI->mData->mModels[0]->GetDofCount() << endl;
  cout << "number of nodes: " << UI->mData->mModels[0]->GetNodeCount() << endl;
  cout << "number of handles: " << UI->mData->mModels[0]->GetHandleCount() << endl;
}

void Solution(void *v)
{
    
    float F = 0;
    for (int i = 0; i < UI->mData->mSelectedModel->GetHandleCount(); i++) {
        F += sqrlen(My_Math_Lib::get_c_value(i));
    }
    cout << "my error" << F << endl;
    float my_Goal = .001 * pow(UI->mData->mSelectedModel->GetHandleCount(), 2);
    while (F > my_Goal) {
        //cout << error << endl;
        Vecd dF_dq = Vecd(UI->mData->mSelectedModel->GetDofCount());
        cout << "Starting for loop for dF_dq" << endl;
        for (int handleIndex = 0; handleIndex < UI->mData->mSelectedModel->GetHandleCount(); handleIndex++) {
            cout<< "On handle #" << handleIndex << endl;
            Vec3d delta_c = My_Math_Lib::get_c_value(handleIndex);
            Matd jacobian = My_Math_Lib::computeJacobian(handleIndex);
            //Matd psd = My_Math_Lib::getJacobianPseudoInverse(jacobian);
            dF_dq += 2 * (trans(jacobian) * delta_c);
        }
        cout<< "Done with for loop" << endl;
//        Vec3d delta_c = My_Math_Lib::get_c_value(0);
//        Matd jacobian = My_Math_Lib::computeJacobian(0);
//        Matd psd = My_Math_Lib::getJacobianPseudoInverse(jacobian);
//        
//        Vecd delta_q = psd * delta_c;
        //cout << "Size of delta_q" << delta_q.Elts() << endl;
        
        Vecd current_q = Vecd(UI->mData->mSelectedModel->GetDofCount());
        
        UI->mData->mSelectedModel->mDofList.GetDofs(&current_q);
        
        bool isCloser = false;
        float alpha = 1;
        while (!isCloser) {
            
            Vecd new_q = current_q - alpha * dF_dq;
            for (int i=0; i < new_q.Elts(); i++) {
                new_q[i] = fmod(new_q[i], 2*2*acos(0.0));
            }
            UI->mData->mSelectedModel->SetDofs(new_q);
            float temp_error = 0;
            for (int i = 0; i < UI->mData->mSelectedModel->GetHandleCount(); i++) {
                temp_error += sqrlen(My_Math_Lib::get_c_value(i));
            }
            if (temp_error < F) {
                F = temp_error;
                isCloser = true;
            } else {
                alpha = alpha - .1;
            }
        }
        cout << "My Error is " << F << endl;
    }
    cout << "Done with Solution" << endl;
}

void Exit(void *v)
{
  exit(0);
}

void LoadC3d(void *v)
{
  if(!UI->mData->mSelectedModel){
    cout << "Load skeleton first";
    return;
  }
  char *params = (char*)v;
  if(!params){
    params = fl_file_chooser("Open File?", "{*.c3d}", "mocap/" );
  }

  if(!params)
    return;
  
  char *c3dFilename = new char[80];
  
  // load single c3d file
 
  C3dFileInfo *openFile = new C3dFileInfo(params);
  openFile->LoadFile();
  UI->mData->mSelectedModel->mOpenedC3dFile = openFile;
  cout << "number of frames in c3d: " << openFile->GetFrameCount() << endl;

  UI->InitControlPanel();
  UI->mGLWindow->mShowConstraints = true;
  UI->mShowConstr_but->value(1);
}
