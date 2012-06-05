import commands
import re

class ControllerException(Exception):
    pass

def load(cname):
    output = commands.getoutput("rosrun pr2_controller_manager pr2_controller_manager load %s"%cname)
    if not output.endswith("successfully"):
        raise ControllerException("load %s failed, gave output %s"%(cname, output))
def stop(cname):
    output = commands.getoutput("rosrun pr2_controller_manager pr2_controller_manager stop %s"%cname)
    if not output.endswith("successfully"):
        raise ControllerException("stop %s failed, gave output %s"%(cname, output))
def start(cname):
    output = commands.getoutput("rosrun pr2_controller_manager pr2_controller_manager start %s"%cname)
    if not output.endswith("successfully"):
        raise ControllerException("start %s failed, gave output %s"%(cname, output))

def get_status(cname,list_output):
    match = re.search("%s \( (\w+) \)"%cname, list_output)
    if match is None: 
        return "unloaded"
    else:
        status =  match.groups()[0]
        assert status == "running" or status == "stopped"
        return status


def switch_to(target, alternative):
    list_output = commands.getoutput("rosrun pr2_controller_manager pr2_controller_manager list")
    target_status = get_status(target,list_output)
    alt_status = get_status(alternative,list_output)

    if alt_status == "running": stop(alternative)
    if target_status == "unloaded": 
        load(target)
        start(target)
    if target_status == "stopped":
        start(target)



        
        

    

