/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#include <cmath>
#include <string>
#include <sstream>
#include <deque>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <opencv2/opencv.hpp>

#include "src/experimentOne_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class ExperimentOne : public RFModule,
                    experimentOne_IDL
{
    string hand_for_computation;
    string hand_for_moving;
    string method;
    string objname;

    vector<cv::Point> contour;
    deque<cv::Point> blob_points;

    RpcClient portBlobRpc;
    RpcClient portOPCrpc;
    RpcClient superqRpc;
    RpcClient graspRpc;
    RpcServer portRpc;

    Mutex mutex;

    Property superq;
    Vector object;
    Vector superq_aux;

    Bottle superq_b;

    bool go_on;
    bool online;
    bool go_home;
    bool filtered;
    bool superq_received;
    bool pose_received;
    bool robot_moving;

    ResourceFinder *rf;

public:

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    Bottle  get_Blob()
    {
        Bottle blob;
        for (size_t i=0; i<blob_points.size(); i++)
        {
            Bottle &b=blob.addList();
            b.addDouble(blob_points[i].x); b.addDouble(blob_points[i].y);
        }

        return blob;
    }

    /************************************************************************/
    bool set_object_name(const string &object_name)
    {
        LockGuard lg(mutex);
        objname=object_name;
        method="name";

        return true;
    }

    /************************************************************************/
    string get_object_name()
    {
        return objname;
    }

    /************************************************************************/
    bool set_hand_for_computation(const string &h)
    {
        LockGuard lg(mutex);
        if ((h=="right") || (h=="left") || (h=="both"))
        {
            hand_for_computation=h;

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    string get_hand_for_computation()
    {
        return hand_for_computation;
    }

    /************************************************************************/
    bool set_hand_for_moving(const string &h)
    {
        LockGuard lg(mutex);
        if ((h=="right") || (h=="left"))
        {
            hand_for_moving=h;

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    string get_hand_for_moving()
    {
        return hand_for_moving;
    }

    /************************************************************************/
    bool go_next()
    {
        go_on=true;

        return true;
    }

    /************************************************************************/
    bool start_from_scratch()
    {
        go_on=false;
        superq_received=false;
        pose_received=false;
        robot_moving=false;

        return true;
    }

    /************************************************************************/
    bool acquire_superq()
    {
        go_on=true;
        superq_received=false;

        return true;
    }

    /************************************************************************/
    bool compute_pose()
    {
        if (superq_received==true)
        {
            go_on=true;
            pose_received=false;

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool grasp_object()
    {
        if (pose_received==true)
        {
            go_on=true;
            robot_moving=false;
            go_home=false;

            return true;
        }
        else
        {
            return false;
        }
    }

    /************************************************************************/
    bool go_back_home()
    {
        go_home=true;

        robot_moving=true;

        go_on=false;

        return true;
    }

    /************************************************************************/
    bool clear_poses()
    {
        Bottle cmd, reply;
        cmd.addString("clear_poses");

        graspRpc.write(cmd, reply);

        return true;
    }

    /************************************************************************/
    bool set_filtered_superq(const string &entry)
    {
        if ((entry=="on") || (entry=="off"))
        {
            filtered=(entry=="on");
            return true;
        }
        else
            return false;          
    }

    /************************************************************************/
    string get_filtered_superq()
    {
        if (filtered)
            return "on";
        else
            return "off";
    }

    /************************************************************************/
    bool getperiod()
    {
        return 0.0;
    }

    /**********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        method=rf.find("method").asString().c_str();
        if (rf.find("method").isNull())
            method="name";

        objname=rf.find("object_name").asString().c_str();
        if (rf.find("object_name").isNull())
            objname="object";

        filtered=(rf.check("filtered", Value("off")).asString()=="on");
        hand_for_computation=rf.check("hand_for_computation", Value("both")).asString();
        hand_for_moving=rf.check("hand_for_moving", Value("right")).asString();

        online=(rf.check("online", Value("off")).asString()=="on");

        if (online==false)
        {
            readSuperq("object", object, 11, this->rf);
            superq=fillProperty(object);
        }

        portBlobRpc.open("/experiment-1/blob:rpc");
        portOPCrpc.open("/experiment-1/OPC:rpc");
        superqRpc.open("/experiment-1/superq:rpc");
        graspRpc.open("/experiment-1/grasp:rpc");
        portRpc.open("/experiment-1/rpc");

        attach(portRpc);

        go_on=false;
        superq_received=false;
        pose_received=false;
        robot_moving=false;

        superq_aux.resize(12,0.0);

        return true;
    }

    /**********************************************************************/
    bool close()
    {
        if (portBlobRpc.asPort().isOpen())
            portBlobRpc.close();
        if (portOPCrpc.asPort().isOpen())
            portOPCrpc.close();
        if (superqRpc.asPort().isOpen())
            superqRpc.close();
        if (graspRpc.asPort().isOpen())
            graspRpc.close();

        return true;
    }

    /**********************************************************************/
    bool updateModule()
    {
        if (online==true)
        {
            if (method=="point")
            {
                blob_points.clear();

                if (contour.size()>0)
                {
                    getBlob();
                }
                else
                {
                    blob_points.push_back(cv::Point(0,0));
                }
            }
            else if (method=="name")
            {
                pointFromName();

                if ((contour.size()>0) )
                {
                    getBlob();
                }
                else
                {
                    blob_points.push_back(cv::Point(0,0));
                }
            }
        }

        if ((go_on==true) && (superq_received==false) && (online==true))
        {
            Bottle cmd;
            cmd.addString("get_superq");

            Bottle &in1=cmd.addList();
        
            for (size_t i=0; i<blob_points.size(); i++)
            {
                Bottle &in=in1.addList();
                in.addDouble(blob_points[i].x);                        
                in.addDouble(blob_points[i].y);
            }

            go_on=false;
            
            // Add 1 instead of 0 if you want the filtered superquadric
            if (filtered)
                cmd.addInt(1);
            else
                cmd.addInt(0);

            superqRpc.write(cmd, superq_b);

            yInfo()<<"Received superquadric: "<<superq_b.toString();

            superq_received=true;
        }
        else if (online==false)
            superq_received=true;

        if ((go_on==true) && (superq_received==true) && (pose_received==false))
        {
            Bottle cmd, reply;
            cmd.addString("get_grasping_pose");

            getBottle(superq_b, cmd);

            cmd.addString(hand_for_computation);

            yInfo()<<"Command asked "<<cmd.toString();

            graspRpc.write(cmd, reply);

            yInfo()<<"Received solution: "<<reply.toString();

            if (reply.size()>0)
                pose_received=true;

            go_on=false;
        }

        if ((go_on==true) && (pose_received==true) && (robot_moving==false))
        {
            Bottle cmd, reply;
            cmd.clear();
            cmd.addString("move");
            cmd.addString(hand_for_moving);

            yInfo()<<"Asked to move: "<<cmd.toString();

            graspRpc.write(cmd, reply);

            if (reply.get(0).asString()=="ok")
            {
                yInfo()<<"The robot is moving";
                robot_moving=true;
            }

            go_on=false;
        }

        if (go_home==true)
        {
            Bottle cmd, reply;
            cmd.clear();
            cmd.addString("go_home");
            cmd.addString(hand_for_moving);

            yInfo()<<"Asked to stop: "<<cmd.toString();

            graspRpc.write(cmd, reply);

            go_home=false;
        }

        return true;
    }

    /***********************************************************************/
    void getBlob()
    {
        Bottle cmd,reply;
        blob_points.clear();
        
        cmd.addString("get_component_around");
        cmd.addInt(contour[0].x); cmd.addInt(contour[0].y);

        if (portBlobRpc.write(cmd,reply))
        {           
            if (Bottle *blob_list=reply.get(0).asList())
            {
                for (int i=0; i<blob_list->size();i++)
                {
                    if (Bottle *blob_pair=blob_list->get(i).asList())
                    {
                        blob_points.push_back(cv::Point(blob_pair->get(0).asInt(),blob_pair->get(1).asInt()));
                    }
                    else
                    {
                        yError()<<"Some problems in blob pixels!";
                    }
                }
            }
            else
            {
                yError()<<"Some problem  in object blob!";
            }
        }
        else
        {
            yError("lbpExtract query is fail!");
        }
    }

    /***********************************************************************/
    void pointFromName()
    {
        Bottle cmd,reply;
        blob_points.clear();
        contour.clear();
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content=cmd.addList();
        Bottle &cond_1=content.addList();
        cond_1.addString("entity");
        cond_1.addString("==");
        cond_1.addString("object");
        content.addString("&&");
        Bottle &cond_2=content.addList();
        cond_2.addString("name");
        cond_2.addString("==");
        cond_2.addString(objname);

        portOPCrpc.write(cmd,reply);
        if(reply.size()>1)
        {
            if(reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *b=reply.get(1).asList())
                {
                    if (Bottle *b1=b->get(1).asList())
                    {
                        cmd.clear();
                        int id=b1->get(0).asInt();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &info=cmd.addList();
                        Bottle &info2=info.addList();
                        info2.addString("id");
                        info2.addInt(id);
                        Bottle &info3=info.addList();
                        info3.addString("propSet");
                        Bottle &info4=info3.addList();
                        info4.addString("position_2d_left");
                    }
                    else
                    {
                        yError("no object id provided by OPC!");
                        contour.clear();
                    }
                }
                else
                {
                    yError("uncorrect reply from OPC!");
                    contour.clear();
                }

                Bottle reply;
                if (portOPCrpc.write(cmd,reply))
                {

                    if (reply.size()>1)
                    {
                        if (reply.get(0).asVocab()==Vocab::encode("ack"))
                        {
                            if (Bottle *b=reply.get(1).asList())
                            {
                                if (Bottle *b1=b->find("position_2d_left").asList())
                                {
                                    cv::Point p1,p2,p;
                                    p1.x=b1->get(0).asInt();
                                    p1.y=b1->get(1).asInt();
                                    p2.x=b1->get(2).asInt();
                                    p2.y=b1->get(3).asInt();
                                    p.x=p1.x+(p2.x-p1.x)/2;
                                    p.y=p1.y+(p2.y-p1.y)/2;
                                    contour.push_back(p);
                                }
                                else
                                {
                                    yError("position_2d_left field not found in the OPC reply!");
                                    contour.clear();
                                }
                            }
                            else
                            {
                                yError("uncorrect reply structure received!");
                                contour.clear();
                            }
                        }
                        else
                        {
                            yError("Failure in reply for object 2D point!");
                            contour.clear();
                        }
                    }
                    else
                    {
                        yError("reply size for 2D point less than 1!");
                        contour.clear();
                    }
                }
                else
                    yError("no reply from second OPC query!");
            }
            else
            {
                yError("Failure in reply for object id!");
                contour.clear();
            }
        }
        else
        {
            yError("reply size for object id less than 1!");
            contour.clear();
        }
    }

    /****************************************************************/
    bool readSuperq(const string &name_obj, Vector &x, const int &dimension, ResourceFinder *rf)
    {
        if (Bottle *b=rf->find(name_obj.c_str()).asList())
        {
            if (b->size()>=dimension)
            {
                for(size_t i=0; i<b->size();i++)
                    x.push_back(b->get(i).asDouble());
            }
            return true;
        }
    }

    /**********************************************************************/
    Property fillProperty(const Vector &sol)
    {
        Property superq;

        Bottle bottle;
        Bottle &b1=bottle.addList();
        b1.addDouble(sol[0]); b1.addDouble(sol[1]); b1.addDouble(sol[2]);
        superq.put("dimensions", bottle.get(0));

        Bottle &b2=bottle.addList();
        b2.addDouble(sol[3]); b2.addDouble(sol[4]);
        superq.put("exponents", bottle.get(1));

        Bottle &b3=bottle.addList();
        b3.addDouble(sol[5]); b3.addDouble(sol[6]); b3.addDouble(sol[7]);
        superq.put("center", bottle.get(2));

        Bottle &b4=bottle.addList();
        Vector orient=dcm2axis(euler2dcm(sol.subVector(8,10)));
        b4.addDouble(orient[0]); b4.addDouble(orient[1]); b4.addDouble(orient[2]); b4.addDouble(orient[3]);
        superq.put("orientation", bottle.get(3));
        return superq;
    }

    /**********************************************************************/
    void getBottle(Bottle &estimated_superq, Bottle &cmd)
    {
        Bottle *all=estimated_superq.get(0).asList();

        for (size_t i=0; i<all->size(); i++)
        {
            Bottle *group=all->get(i).asList();
            if (group->get(0).asString() == "dimensions")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
            }
            else if (group->get(0).asString() == "exponents")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[3]=dim->get(0).asDouble(); superq_aux[4]=dim->get(1).asDouble();
            }
            else if (group->get(0).asString() == "center")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[5]=dim->get(0).asDouble(); superq_aux[6]=dim->get(1).asDouble(); superq_aux[7]=dim->get(2).asDouble();
            }
            else if (group->get(0).asString() == "orientation")
            {
                 Bottle *dim=group->get(1).asList();

                 superq_aux[8]=dim->get(0).asDouble(); superq_aux[9]=dim->get(1).asDouble(); superq_aux[10]=dim->get(2).asDouble(); superq_aux[11]=dim->get(3).asDouble();
            }

        }
                
        /*Bottle *dim=estimated_superq.find("dimensions").asList();

        if (!estimated_superq.find("dimensions").isNull())
        {
            superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
        }

        Bottle *shape=estimated_superq.find("exponents").asList();

        if (!estimated_superq.find("exponents").isNull())
        {
            superq_aux[3]=shape->get(0).asDouble(); superq_aux[4]=shape->get(1).asDouble();
        }

        Bottle *exp=estimated_superq.find("exponents").asList();

        if (!estimated_superq.find("exponents").isNull())
        {
            superq_aux[3]=exp->get(0).asDouble(); superq_aux[4]=exp->get(1).asDouble();
        }

        Bottle *center=estimated_superq.find("center").asList();

        if (!estimated_superq.find("center").isNull())
        {
            superq_aux[5]=center->get(0).asDouble(); superq_aux[6]=center->get(1).asDouble(); superq_aux[7]=center->get(2).asDouble();
        }

        Bottle *orientation=estimated_superq.find("orientation").asList();

        if (!estimated_superq.find("orientation").isNull())
        {
            Vector axis(4,0.0);
            axis[0]=orientation->get(0).asDouble(); axis[1]=orientation->get(1).asDouble(); axis[2]=orientation->get(2).asDouble(); axis[3]=orientation->get(3).asDouble();
            superq_aux.setSubvector(8,axis);
        }*/

        Bottle &b1=cmd.addList();
        Bottle &b2=b1.addList();
        b2.addString("dimensions");
        Bottle &b2l=b2.addList();
        b2l.addDouble(superq_aux[0]); b2l.addDouble(superq_aux[1]); b2l.addDouble(superq_aux[2]);

        Bottle &b3=b1.addList();
        b3.addString("exponents");
        Bottle &b3l=b3.addList();
        b3l.addDouble(superq_aux[3]); b3l.addDouble(superq_aux[4]);

        Bottle &b4=b1.addList();
        b4.addString("center");
        Bottle &b4l=b4.addList();
        b4l.addDouble(superq_aux[5]); b4l.addDouble(superq_aux[6]); b4l.addDouble(superq_aux[7]);

        Bottle &b5=b1.addList();
        b5.addString("orientation");
        Bottle &b5l=b5.addList();
        b5l.addDouble(superq_aux[8]); b5l.addDouble(superq_aux[9]); b5l.addDouble(superq_aux[10]); b5l.addDouble(superq_aux[11]);
    }
};

/**********************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    ExperimentOne mod;
    ResourceFinder rf;
    rf.setDefaultContext("experiment-1");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
