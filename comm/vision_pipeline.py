from pipeline import Pipeline
import os


def make_vision_pipeline(downsample, classifier, init_period=0):    
    
    if init_period == 0: init_period = 9999
    
    P = Pipeline()
    
    P.add_topic("kinect")
    P.add_topic("labels")
    P.add_topic("images")
    P.add_topic("rope_pts")
    P.add_topic("towel_pts")
    P.add_topic("rope_init",dont_clean=True)
    P.add_topic("rope_model")
    P.add_topic("towel_model")
    
    P.add_file("once/table_corners.txt")
    P.add_file("once/roi_mask.png")

    P.add_program("write_pcds", "comm_write_pcds kinect -d %i"%downsample, [], ["kinect"])
    P.add_program("pcd2bmp", "comm_pcd2bmp.py kinect images", ["kinect"], ["images"])
    P.add_program("apply_classifier", "comm_apply_classifier_par.py images labels %s --procs=3 --mask=once/roi_mask.png"%classifier, ["images","once/roi_mask.png"], ["labels"])
    P.add_program("rope_downsampler", "comm_downsample_clouds -c kinect -l labels -n 1 2 -o rope_pts",["kinect","labels"],["rope_pts"])
    P.add_program("towel_downsampler", "comm_downsample_clouds -c kinect -l labels -n 1 -o towel_pts -v .025",["kinect","labels"],["towel_pts"])
    P.add_program("rope_initializer", "loop_initialize_rope_lite.py kinect labels rope_init %.2f"%init_period,["kinect","labels"],["rope_init"])    
    P.add_program("track_human_rope", "test_new_tracking2 --kp=1 --kd=0 --showEst=0 --showObs=0 --showKinect=1", ["kinect","labels","rope_init","rope_pts","once/table_corners.txt"], ["rope_model"])


    P.add_program("track_human_cloth", "test_new_towel_tracking --objType=towel --kp=300 --kd=20 --gravity=0 --towelRes=.5 --showEst=0 --showObs=0 --showLines=0 --showKinect=0 --towelStiffness=.1", ["kinect","labels","towel_pts","once/table_corners.txt"],["towel_model"])
    
    P.add_program("make_roi", "make_roi.py -i images/data000000000000.bmp -o once/roi_mask.png", ["images"], ["once/roi_mask.png"])
    P.add_program("get_table", "comm_get_table", ["kinect"], ["once/table_corners.txt"])
    
    
    P.env = os.environ    
               
    return P

def make_towel_pipeline(downsample=10):
    
    P = Pipeline()
    P.add_topic("kinect")
    P.add_topic("towel_pts")    
    P.add_file("once/table_corners.txt")
    P.add_topic("towel_model")
    P.add_program("get_table", "comm_get_table", ["kinect"],["once/table_corners.txt"])
    P.add_program("write_pcds", "comm_write_pcds kinect -d %i"%downsample, [], ["kinect"])
    P.add_program("towel_preproc", "comm_towel_preproc", ["kinect","once/table_corners.txt"], ["towel_pts"])
    P.add_program("towel_tracker", "comm_track_towel --kp=200 --kd=0 --gravity=0", ["kinect", "once/table_corners.txt", "towel_pts"], ["towel_model"])

    P.env = os.environ    
                       
    return P

def make_robot_rope_pipeline(classifier, downsample=5, init_period=0):
    
    if init_period == 0: init_period = 9999
    
    P = Pipeline()
    
    P.add_topic("kinect")
    P.add_topic("labels")
    P.add_topic("images")
    P.add_topic("rope_pts")
    P.add_topic("rope_init",dont_clean=True)
    P.add_topic("rope_model")
    P.add_topic("joint_states",async=True)
    
    P.add_file("once/table_corners.txt")
    P.add_file("once/roi_mask.png")

    P.add_program("write_pcds", "comm_write_pcds kinect -d %i"%downsample, [], ["kinect"])
    P.add_program("pcd2bmp", "comm_pcd2bmp.py kinect images", ["kinect"], ["images"])
    P.add_program("apply_classifier", "comm_apply_classifier_par.py images labels %s --procs=3 --mask=once/roi_mask.png"%classifier, ["images","once/roi_mask.png"], ["labels"])
    #P.add_program("apply_classifier", "comm_apply_classifier.py images labels %s --mask=once/roi_mask.png"%classifier, ["images","once/roi_mask.png"], ["labels"])

    P.add_program("downsampler", "comm_downsample_clouds -c kinect -l labels -n 1 2 -o rope_pts",["kinect","labels"],["rope_pts"])
    P.add_program("initializer", "loop_initialize_rope_lite.py kinect labels rope_init %.2f"%init_period,["kinect","labels"],["rope_init"])    
    P.add_program("track_rope_with_robot", "track_rope_with_robot --kp=1 --kd=0 --showEst=0 --showObs=0 --showKinect=1", ["kinect","labels","rope_init","rope_pts","joint_states","once/table_corners.txt"], ["rope_model"])    
    P.add_program("make_roi", "make_roi.py -i images/data000000000000.bmp -o once/roi_mask.png", ["images"], ["once/roi_mask.png"])
    P.add_program("get_table", "comm_get_table", ["kinect"], ["once/table_corners.txt"])
    P.add_program("write_joints", "publish_ros_topic.py /joint_states --out joint_states -d 5", [], ["joint_states"])
        
    P.env = os.environ    
               
    return P

def make_robot_rope_pipeline2(classifier, downsample=5, init_period=0):
    
    if init_period == 0: init_period = 9999
    
    P = Pipeline()
    
    P.add_topic("kinect")
    P.add_topic("rope_pts")
    P.add_topic("rope_init",dont_clean=True)
    P.add_topic("rope_model")
    P.add_topic("joint_states",async=True)
    P.add_topic("base_pose", async=True)
    
    P.add_file("once/table_corners.txt")
    P.add_file("once/transform.txt")
    P.add_program("write_pcds", "comm_write_pcds kinect -d %i"%downsample, [], ["kinect"])

    P.add_program("initializer", "loop_initialize_rope_lite.py rope_pts rope_init %.2f"%init_period,["rope_pts"],["rope_init"])    

    P.add_program("rope_preproc", "comm_rope_preproc", ["kinect","once/table_corners.txt"], ["rope_pts"])


    P.add_program("track_rope_with_robot", "track_rope_with_robot --scale=10 --kp=1 --kd=0 --outlierParam=.25 --showEst=0 --showObs=0 --showKinect=1", ["kinect","rope_init","rope_pts","joint_states","once/table_corners.txt","once/transform.txt"], ["rope_model"])    
    P.add_program("get_table", "comm_get_table", ["kinect"], ["once/table_corners.txt"])
    P.add_program("write_joints", "publish_ros_topic.py /joint_states --out joint_states -d 5", [], ["joint_states"])
    P.add_program("write_base_pose", "tf_to_comm.py /base_footprint /base_footprint base_pose --hz=25", [], ["base_pose"])
        
        
    P.env = os.environ    
               
    return P


def make_human_rope_pipeline(downsample=5, init_period=0):
    
    if init_period == 0: init_period = 9999
    
    P = Pipeline()
    
    P.add_topic("kinect")
    P.add_topic("rope_pts")
    P.add_topic("rope_init",dont_clean=True)
    P.add_topic("rope_model")
    
    P.add_file("once/table_corners.txt")
    P.add_program("write_pcds", "comm_write_pcds kinect -d %i"%downsample, [], ["kinect"])

    P.add_program("initializer", "loop_initialize_rope_lite.py rope_pts rope_init %.2f"%init_period,["rope_pts"],["rope_init"])    

    P.add_program("rope_preproc", "comm_rope_preproc", ["kinect","once/table_corners.txt"], ["rope_pts"])


    P.add_program("track_rope", "track_rope --scale=10 --kp=.1 --kd=0 --outlierParam=.2 --showEst=0 --showObs=1 --showKinect=1", ["kinect","rope_init","rope_pts","once/table_corners.txt"], ["rope_model"])    
    P.add_program("get_table", "comm_get_table", ["kinect"], ["once/table_corners.txt"])
        
    P.env = os.environ    
               
    return P
