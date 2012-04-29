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

    float error = sqrlen(My_Math_Lib::get_c_value());
    Vec3d delta_c = My_Math_Lib::get_c_value();
    cout << "my error" << error << endl;
    while (error > .001) {
        //cout << error << endl;
        Matd jacobian = My_Math_Lib::computeJacobian();
        Matd psd = My_Math_Lib::getJacobianPseudoInverse(jacobian);
        cout << "My PSD is :" << psd << endl;
        Vecd delta_q = psd * delta_c;
        cout << "My delta_q: " << delta_q << endl;
        
        Vecd current_q = Vecd(UI->mData->mSelectedModel->GetDofCount());
        
        UI->mData->mSelectedModel->mDofList.GetDofs(&current_q);
        
        bool isCloser = false;
        float alpha = .5;
        while (!isCloser) {
            Vecd new_q = current_q + alpha * delta_q;
            UI->mData->mSelectedModel->SetDofs(new_q);
            float temp_error = sqrlen(My_Math_Lib::get_c_value());
            if (temp_error < error) {
                error = temp_error;
                isCloser = true;
                cout << "My New q = " << new_q << endl;
            } else {
                alpha = alpha - .01;
            }
        }
        cout << "My Error is " << error << endl;
        Vec3d new_delta_c = My_Math_Lib::get_c_value();
        delta_c = new_delta_c;
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
