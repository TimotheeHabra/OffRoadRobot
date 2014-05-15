/*
 * JNI (java virtual machine so as to access to the 3D animation) functions
 *
 * authors: Nicolas Van der Noot and Nicolas Docquier
 */
#if defined(JNI) & defined(REAL_TIME)

#include "jni_functions.h"
#include "project_info.h"

/* 
 * This function instantiates a java virtual machine so as to access 
 * to the 3D animation facilities of MBSysPad.
 */
JNIEnv* create_vm(void)
{
    int res;

	JavaVM* jvm;
	JNIEnv* env;
	JavaVMInitArgs args;
	JavaVMOption options[2];
    
	// the following parameter should be adapted depending on the version of the JDK
	args.version = JNI_VERSION;
    // number of argument to pass to the jvm
	args.nOptions = 2;
	// path to the jar file containing mbsyspad bynary code
	options[0].optionString = "-Djava.class.path="JAR_PATH;
	// path to the folder containing additional libraries, in particular the java3D native file (libj3dcore-ogl.so for linux)
	options[1].optionString = "-Djava.library.path="JAVA3D_PATH;

	args.options = options;
	args.ignoreUnrecognized = JNI_FALSE;

	res = JNI_CreateJavaVM(&jvm, (void **)&env, &args);
    
    if (res < 0)
    {
        switch ( res )
        {
            case -1:
                fprintf(stderr, "/* unknown error */\n");
                break;
            case -2:
                fprintf(stderr, "/* thread detached from the VM */\n");
                break;
            case -3:
                fprintf(stderr, "/* JNI version error */\n");
                break;
            case -4:
                fprintf(stderr, "/* not enough memory */\n");
                break;
            case -5:
                fprintf(stderr, "/* VM already created */\n");
                break;
            case -6:
                fprintf(stderr, "/* invalid arguments */\n");
                break;
            default:
                fprintf(stderr, "Can't create Java VM\n");
        }
        exit(1);
    }
    
	return env;
}

/*
 * Initializes the JNI structure
 */
JNI_struct* init_jni(MBSdataStruct *MBSdata)
{

    JNI_struct *jni_struct;

    // - - - - - - - - - - - - - - - - -
    // declaration for 3D view with JNI

    jclass animFrameClass,  PadModelClass, MbsModelJ3DClass;
    jmethodID constructor;
    jmethodID loadMethod, setLocMethod;
    jmethodID getModelMethod, getMbs3DMethod, setVpMethod;
    jobject model, model3D;
    jstring mbsFilename;
    jdoubleArray doubleArrayArg;

    JNIEnv* env;
    jmethodID updateJointMethod;
    jobject obj;

    // - - - - - - - - - - - - - - - - - - - - - - -
    // initialise jni variable for 3D visualisation

    env = create_vm();

    animFrameClass = (*env)->FindClass(env, "be/robotran/test/LiveSimAnimFrame");

    // get the reference to the constructor of this class
    constructor = (*env)->GetMethodID(env, animFrameClass, "<init>", "()V");

    // get the reference to the load method which load the *.mbs file
    loadMethod = (*env)->GetMethodID(env, animFrameClass, "load", "(Ljava/lang/String;)V");
   
    // get the reference to the updateJoints method which update the 3D view during simulation
    updateJointMethod = (*env)->GetMethodID(env, animFrameClass, "updateJoints", "([D)V");
 
    // get the reference to the setLocation method 
    setLocMethod = (*env)->GetMethodID(env, animFrameClass, "setLocation", "(II)V");
    
    // create a jni string containing the path to the *.mbs file to load
    mbsFilename = (*env)->NewStringUTF(env, MBS_FILE);
    
    // create an instance of the 3D view
    obj = (*env)->NewObject(env, animFrameClass, constructor);
    
    // load the mbsfilename -> keyboard inputs problem !!!
    (*env)->CallObjectMethod(env, obj, loadMethod, mbsFilename);

    // Viewpoint
    // get the reference to the class
    PadModelClass    = (*env)->FindClass(env, "be/robotran/mbsyspad/PadModel");
    MbsModelJ3DClass = (*env)->FindClass(env, "be/robotran/mbs3Dviewer/j3Dviewer/MbsModelJ3D");

    // get the reference to the methods
    getModelMethod = (*env)->GetMethodID(env, animFrameClass, "getModel", "()Lbe/robotran/mbsyspad/PadModel;");
    getMbs3DMethod = (*env)->GetMethodID(env, PadModelClass, "getMbs3D", "()Lbe/robotran/mbs3Dviewer/viewerInterface/MbsModel3D;");
    setVpMethod    = (*env)->GetMethodID(env, MbsModelJ3DClass, "setViewpoint", "(I)V");

    model   = (*env)->CallObjectMethod(env, obj, getModelMethod);
    model3D = (*env)->CallObjectMethod(env, model, getMbs3DMethod);
    (*env)->CallObjectMethod(env, model3D, setVpMethod, -1);

    // move the frame
    (*env)->CallObjectMethod(env, obj, setLocMethod, 100,100);

    // instantiate a jni array of double
    doubleArrayArg = (*env)->NewDoubleArray(env, MBSdata->njoint);

    // copy the current values of MBSdata->q to the jni array
    (*env)->SetDoubleArrayRegion(env, doubleArrayArg, 0 , MBSdata->njoint, MBSdata->q+1);

    // update the 3D view
    (*env)->CallObjectMethod(env, obj, updateJointMethod, doubleArrayArg);

    // JNI structure
    jni_struct = (JNI_struct*) malloc(sizeof(JNI_struct));

    jni_struct->env               = env;
    jni_struct->updateJointMethod = updateJointMethod;
    jni_struct->obj               = obj;
    jni_struct->setVpMethod       = setVpMethod;
    jni_struct->model3D           = model3D;

    return jni_struct; 
}

/*
 * Close memory
 */
void free_jni(JNI_struct *jni_struct)
{
    free(jni_struct);
}

/*
 * Update the 3D view with jni
 */
void update_jni(JNI_struct *jni_struct, MBSdataStruct *s, Simu_real_time *real_time)
{
    // variables declarationb
    jdoubleArray doubleArrayArg;
    JNIEnv* env;

    env = jni_struct->env;

    // instantiate a jni array of double
    doubleArrayArg = (*env)->NewDoubleArray(env, s->njoint);

    // copy the current values of MBSdata->q to the jni array
    (*env)->SetDoubleArrayRegion(env, doubleArrayArg, 0 , s->njoint, s->q+1);

    // adapt the viewpoint
    if (real_time->change_viewpoint)
    {
        real_time->change_viewpoint = 0;

        real_time->viewpoint_nb++;
        if (real_time->viewpoint_nb >= NB_VIEWPOINTS)
        {
            real_time->viewpoint_nb = START_VIEWPOINT;
        }

        (*env)->CallObjectMethod(env, jni_struct->model3D, jni_struct->setVpMethod, real_time->viewpoint_nb);
    }
    
    // update the 3D view
    (*env)->CallObjectMethod(env, jni_struct->obj, jni_struct->updateJointMethod, doubleArrayArg);
}

#endif
