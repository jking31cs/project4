// generated by Fast Light User Interface Designer (fluid) version 1.0103

#ifndef phylterui_h
#define phylterui_h
#include <FL/Fl.H>
#include <vector>
class Phylter_Fl_Gl_Window;
class AnalyzerWindow;
#include "UICallback.h"
#include <FL/FileName.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Scroll.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Browser.H>
#include <FL/Fl_Progress.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Counter.H>
#include <FL/Fl_Roller.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Return_Button.H>
#include <FL/Fl_Help_View.H>
#include <FL/Fl_Light_Button.H>
#include <FL/Fl_Value_Input.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Slider.H>

class PhylterUI {
public:
  Fl_Cursor mCursor;
  dirent** mList;
  std::vector<Fl_Value_Slider*> mDofs_sli;
  PhylterUI();
  Fl_Window *mFLWindow;
  Phylter_Fl_Gl_Window *mGLWindow;
  Fl_Scroll *mFuncList_scr;
  Fl_Scroll *mDofList_scr;
  Fl_Scroll *mFpsScrollPane;
  Fl_Input *mCommandLine_inp;
private:
  inline void cb_mCommandLine_inp_i(Fl_Input*, void*);
  static void cb_mCommandLine_inp(Fl_Input*, void*);
public:
  Fl_Browser *mGeneralInfo_out;
  Fl_Progress *mProgress_pro;
  Fl_Value_Slider *mFrame_sli;
private:
  inline void cb_mFrame_sli_i(Fl_Value_Slider*, void*);
  static void cb_mFrame_sli(Fl_Value_Slider*, void*);
public:
  Fl_Value_Slider *mBegin_sli;
private:
  inline void cb_mBegin_sli_i(Fl_Value_Slider*, void*);
  static void cb_mBegin_sli(Fl_Value_Slider*, void*);
public:
  Fl_Value_Slider *mEnd_sli;
private:
  inline void cb_mEnd_sli_i(Fl_Value_Slider*, void*);
  static void cb_mEnd_sli(Fl_Value_Slider*, void*);
public:
  Fl_Button *mLoop_but;
private:
  inline void cb_mLoop_but_i(Fl_Button*, void*);
  static void cb_mLoop_but(Fl_Button*, void*);
public:
  Fl_Counter *mFrameCounter_cou;
private:
  inline void cb_mFrameCounter_cou_i(Fl_Counter*, void*);
  static void cb_mFrameCounter_cou(Fl_Counter*, void*);
public:
  Fl_Button *mPlay_but;
private:
  inline void cb_mPlay_but_i(Fl_Button*, void*);
  static void cb_mPlay_but(Fl_Button*, void*);
public:
  Fl_Roller *mSpeed_rol;
private:
  inline void cb_mSpeed_rol_i(Fl_Roller*, void*);
  static void cb_mSpeed_rol(Fl_Roller*, void*);
  static Fl_Menu_Item menu_[];
  inline void cb_Load_i(Fl_Menu_*, void*);
  static void cb_Load(Fl_Menu_*, void*);
  inline void cb_Load1_i(Fl_Menu_*, void*);
  static void cb_Load1(Fl_Menu_*, void*);
  inline void cb_Load2_i(Fl_Menu_*, void*);
  static void cb_Load2(Fl_Menu_*, void*);
  inline void cb_Load3_i(Fl_Menu_*, void*);
  static void cb_Load3(Fl_Menu_*, void*);
  inline void cb_Save_i(Fl_Menu_*, void*);
  static void cb_Save(Fl_Menu_*, void*);
  inline void cb_Save1_i(Fl_Menu_*, void*);
  static void cb_Save1(Fl_Menu_*, void*);
  inline void cb_E_i(Fl_Menu_*, void*);
  static void cb_E(Fl_Menu_*, void*);
  inline void cb_Keyframe_i(Fl_Menu_*, void*);
  static void cb_Keyframe(Fl_Menu_*, void*);
  inline void cb_Set_i(Fl_Menu_*, void*);
  static void cb_Set(Fl_Menu_*, void*);
  inline void cb_Delete_i(Fl_Menu_*, void*);
  static void cb_Delete(Fl_Menu_*, void*);
  inline void cb_Record_i(Fl_Menu_*, void*);
  static void cb_Record(Fl_Menu_*, void*);
  inline void cb_Still_i(Fl_Menu_*, void*);
  static void cb_Still(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *mShowModel_ite;
private:
  inline void cb_mShowModel_ite_i(Fl_Menu_*, void*);
  static void cb_mShowModel_ite(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *mShowConstr_ite;
private:
  inline void cb_mShowConstr_ite_i(Fl_Menu_*, void*);
  static void cb_mShowConstr_ite(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *mShowMarker_ite;
private:
  inline void cb_mShowMarker_ite_i(Fl_Menu_*, void*);
  static void cb_mShowMarker_ite(Fl_Menu_*, void*);
  inline void cb_Sliders_i(Fl_Menu_*, void*);
  static void cb_Sliders(Fl_Menu_*, void*);
  inline void cb_Interactive_i(Fl_Menu_*, void*);
  static void cb_Interactive(Fl_Menu_*, void*);
  inline void cb_Solve_i(Fl_Menu_*, void*);
  static void cb_Solve(Fl_Menu_*, void*);
  inline void cb_Terminate_i(Fl_Menu_*, void*);
  static void cb_Terminate(Fl_Menu_*, void*);
  inline void cb_Preprocess_i(Fl_Menu_*, void*);
  static void cb_Preprocess(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *mContent_ite;
private:
  inline void cb_mContent_ite_i(Fl_Menu_*, void*);
  static void cb_mContent_ite(Fl_Menu_*, void*);
  inline void cb_About_i(Fl_Menu_*, void*);
  static void cb_About(Fl_Menu_*, void*);
public:
  Fl_Button *mDefault_but;
private:
  inline void cb_mDefault_but_i(Fl_Button*, void*);
  static void cb_mDefault_but(Fl_Button*, void*);
public:
  Fl_Button *mModel_but;
private:
  inline void cb_mModel_but_i(Fl_Button*, void*);
  static void cb_mModel_but(Fl_Button*, void*);
public:
  Fl_Button *mMotion_but;
private:
  inline void cb_mMotion_but_i(Fl_Button*, void*);
  static void cb_mMotion_but(Fl_Button*, void*);
public:
  Fl_Button *mConstr_but;
private:
  inline void cb_mConstr_but_i(Fl_Button*, void*);
  static void cb_mConstr_but(Fl_Button*, void*);
public:
  Fl_Button *mSaveMotion_but;
private:
  inline void cb_mSaveMotion_but_i(Fl_Button*, void*);
  static void cb_mSaveMotion_but(Fl_Button*, void*);
public:
  Fl_Button *mSaveConstr_but;
private:
  inline void cb_mSaveConstr_but_i(Fl_Button*, void*);
  static void cb_mSaveConstr_but(Fl_Button*, void*);
public:
  Fl_Button *mSelectKey_but;
private:
  inline void cb_mSelectKey_but_i(Fl_Button*, void*);
  static void cb_mSelectKey_but(Fl_Button*, void*);
public:
  Fl_Button *mDeleteKey_but;
private:
  inline void cb_mDeleteKey_but_i(Fl_Button*, void*);
  static void cb_mDeleteKey_but(Fl_Button*, void*);
public:
  Fl_Button *mRecordMotion_but;
private:
  inline void cb_mRecordMotion_but_i(Fl_Button*, void*);
  static void cb_mRecordMotion_but(Fl_Button*, void*);
public:
  Fl_Button *mStillShot_but;
private:
  inline void cb_mStillShot_but_i(Fl_Button*, void*);
  static void cb_mStillShot_but(Fl_Button*, void*);
public:
  Fl_Button *mShowModel_but;
private:
  inline void cb_mShowModel_but_i(Fl_Button*, void*);
  static void cb_mShowModel_but(Fl_Button*, void*);
public:
  Fl_Button *mShowConstr_but;
private:
  inline void cb_mShowConstr_but_i(Fl_Button*, void*);
  static void cb_mShowConstr_but(Fl_Button*, void*);
public:
  Fl_Button *mShowMarker_but;
private:
  inline void cb_mShowMarker_but_i(Fl_Button*, void*);
  static void cb_mShowMarker_but(Fl_Button*, void*);
public:
  Fl_Button *mInteract_but;
private:
  inline void cb_mInteract_but_i(Fl_Button*, void*);
  static void cb_mInteract_but(Fl_Button*, void*);
public:
  Fl_Button *mSpacetime_but;
private:
  inline void cb_mSpacetime_but_i(Fl_Button*, void*);
  static void cb_mSpacetime_but(Fl_Button*, void*);
public:
  Fl_Button *mTerminate_but;
private:
  inline void cb_mTerminate_but_i(Fl_Button*, void*);
  static void cb_mTerminate_but(Fl_Button*, void*);
public:
  Fl_Button *mAbout_but;
private:
  inline void cb_mAbout_but_i(Fl_Button*, void*);
  static void cb_mAbout_but(Fl_Button*, void*);
public:
  Fl_Menu_Button *mPhylterList;
  static Fl_Menu_Item menu_mPhylterList[];
  static Fl_Menu_Item *mPhConstr_item;
  static Fl_Menu_Item *mPhPose_item;
  static Fl_Menu_Item *mPhLin_item;
  static Fl_Menu_Item *mPhAng_item;
  static Fl_Menu_Item *mPhSmooth_item;
  static Fl_Menu_Item *mPhMuscle_item;
  Fl_Menu_Button *mFileList;
private:
  inline void cb_mFileList_i(Fl_Menu_Button*, void*);
  static void cb_mFileList(Fl_Menu_Button*, void*);
public:
  AnalyzerWindow *mGraphWindow;
  Fl_Window *mAboutWindow;
  Fl_Output *mAbout_text_out;
  Fl_Return_Button *mReturn_abt_but;
private:
  inline void cb_mReturn_abt_but_i(Fl_Return_Button*, void*);
  static void cb_mReturn_abt_but(Fl_Return_Button*, void*);
public:
  Fl_Window *mContentWindow;
  Fl_Help_View *mHelpView;
private:
  inline void cb_OK_i(Fl_Return_Button*, void*);
  static void cb_OK(Fl_Return_Button*, void*);
public:
  void Show();
  void CreateDofSliderWindow();
  Fl_Window *mDofSliderWindow;
  Fl_Window* CreateKeyframeSettingWindow();
  Fl_Window *mKeyframeSetWindow;
  Fl_Input *mFrames_inp;
  Fl_Input *mTime_inp;
  Fl_Light_Button *mSample_but;
private:
  inline void cb_mSample_but_i(Fl_Light_Button*, void*);
  static void cb_mSample_but(Fl_Light_Button*, void*);
public:
  Fl_Light_Button *mBSpline_but;
private:
  inline void cb_mBSpline_but_i(Fl_Light_Button*, void*);
  static void cb_mBSpline_but(Fl_Light_Button*, void*);
public:
  Fl_Light_Button *mSplitSpline_but;
private:
  inline void cb_mSplitSpline_but_i(Fl_Light_Button*, void*);
  static void cb_mSplitSpline_but(Fl_Light_Button*, void*);
public:
  Fl_Input *mContPoint_inp;
  Fl_Return_Button *mReturn_key_but;
private:
  inline void cb_mReturn_key_but_i(Fl_Return_Button*, void*);
  static void cb_mReturn_key_but(Fl_Return_Button*, void*);
public:
  Fl_Button *mCancel_key_but;
private:
  inline void cb_mCancel_key_but_i(Fl_Button*, void*);
  static void cb_mCancel_key_but(Fl_Button*, void*);
public:
  Fl_Light_Button *mSubdivide_but;
private:
  inline void cb_mSubdivide_but_i(Fl_Light_Button*, void*);
  static void cb_mSubdivide_but(Fl_Light_Button*, void*);
public:
  Fl_Window* CreateDetectorWindow();
  Fl_Window *mDetectionWindow;
  Fl_Value_Input *mMinFrame_det_val;
private:
  inline void cb_mMinFrame_det_val_i(Fl_Value_Input*, void*);
  static void cb_mMinFrame_det_val(Fl_Value_Input*, void*);
public:
  Fl_Value_Input *mEigen_det_val;
private:
  inline void cb_mEigen_det_val_i(Fl_Value_Input*, void*);
  static void cb_mEigen_det_val(Fl_Value_Input*, void*);
public:
  Fl_Value_Input *mDetectRange_det_val;
private:
  inline void cb_mDetectRange_det_val_i(Fl_Value_Input*, void*);
  static void cb_mDetectRange_det_val(Fl_Value_Input*, void*);
public:
  Fl_Value_Input *mSlopeRange_det_val;
private:
  inline void cb_mSlopeRange_det_val_i(Fl_Value_Input*, void*);
  static void cb_mSlopeRange_det_val(Fl_Value_Input*, void*);
public:
  Fl_Value_Input *mMinFlight_det_val;
private:
  inline void cb_mMinFlight_det_val_i(Fl_Value_Input*, void*);
  static void cb_mMinFlight_det_val(Fl_Value_Input*, void*);
public:
  Fl_Check_Button *mPosition_det_che;
private:
  inline void cb_mPosition_det_che_i(Fl_Check_Button*, void*);
  static void cb_mPosition_det_che(Fl_Check_Button*, void*);
public:
  Fl_Check_Button *mLineSlide_det_che;
private:
  inline void cb_mLineSlide_det_che_i(Fl_Check_Button*, void*);
  static void cb_mLineSlide_det_che(Fl_Check_Button*, void*);
public:
  Fl_Check_Button *mPlaneSlide_det_che;
private:
  inline void cb_mPlaneSlide_det_che_i(Fl_Check_Button*, void*);
  static void cb_mPlaneSlide_det_che(Fl_Check_Button*, void*);
public:
  Fl_Return_Button *mReturn_det_but;
  Fl_Button *mCancel_det_but;
  Fl_Check_Button *mToe_det_che;
private:
  inline void cb_mToe_det_che_i(Fl_Check_Button*, void*);
  static void cb_mToe_det_che(Fl_Check_Button*, void*);
public:
  Fl_Light_Button *mSoftConstr_det_but;
  Fl_Value_Input *mMaxTranslation_det_val;
private:
  inline void cb_mMaxTranslation_det_val_i(Fl_Value_Input*, void*);
  static void cb_mMaxTranslation_det_val(Fl_Value_Input*, void*);
public:
  Fl_Window* CreatePostureWindow();
  Fl_Window *mPostureWindow;
  Fl_Return_Button *mReturn_pos_but;
  Fl_Button *mCancel_pos_but;
  Fl_Light_Button *mSoftConstr_pos_but;
  Fl_Window* CreateLinMomentWindow();
  Fl_Window *mLinWindow;
  Fl_Slider *mGamma_ver_scr;
private:
  inline void cb_mGamma_ver_scr_i(Fl_Slider*, void*);
  static void cb_mGamma_ver_scr(Fl_Slider*, void*);
public:
  Fl_Slider *mGamma_hor_scr;
private:
  inline void cb_mGamma_hor_scr_i(Fl_Slider*, void*);
  static void cb_mGamma_hor_scr(Fl_Slider*, void*);
public:
  Fl_Slider *mLambda_hor_scr;
private:
  inline void cb_mLambda_hor_scr_i(Fl_Slider*, void*);
  static void cb_mLambda_hor_scr(Fl_Slider*, void*);
public:
  Fl_Slider *mLambda_ver_scr;
private:
  inline void cb_mLambda_ver_scr_i(Fl_Slider*, void*);
  static void cb_mLambda_ver_scr(Fl_Slider*, void*);
public:
  Fl_Return_Button *mReturn_lin_but;
  Fl_Button *mCancel_lin_but;
  Fl_Light_Button *mSoftConstr_lin_but;
  Fl_Window* CreateAngMomentWindow();
  Fl_Window *mAngWindow;
  Fl_Slider *mAlpha_ver_scr;
private:
  inline void cb_mAlpha_ver_scr_i(Fl_Slider*, void*);
  static void cb_mAlpha_ver_scr(Fl_Slider*, void*);
public:
  Fl_Slider *mAlpha_hor_scr;
private:
  inline void cb_mAlpha_hor_scr_i(Fl_Slider*, void*);
  static void cb_mAlpha_hor_scr(Fl_Slider*, void*);
public:
  Fl_Slider *mBeta_hor_scr;
private:
  inline void cb_mBeta_hor_scr_i(Fl_Slider*, void*);
  static void cb_mBeta_hor_scr(Fl_Slider*, void*);
public:
  Fl_Slider *mBeta_ver_scr;
private:
  inline void cb_mBeta_ver_scr_i(Fl_Slider*, void*);
  static void cb_mBeta_ver_scr(Fl_Slider*, void*);
public:
  Fl_Return_Button *mReturn_ang_but;
  Fl_Button *mCancel_ang_but;
  Fl_Light_Button *mSoftConstr_ang_but;
  void InitKeyframeSetting();
  void InitSliders();
  void InitControlPanel();
  void InitFileList();
  void InitFuncList();
  void InitDofList();
  void DeleteSliderWindow();
  void DeleteKeyframeSettingWindow();
  void DeleteFileList();
  void DeleteFuncList();
  void DeleteDofList();
  std::vector<Fl_Counter*> mFPS_count;
  void InitFpsSliders();
};
#endif
