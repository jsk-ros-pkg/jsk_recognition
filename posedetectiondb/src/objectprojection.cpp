#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

using namespace OpenRAVE;
using namespace std;

/// vcamerapoints is a vector of 2D points in camera space (multiplied by K^-1)
/// tcamera is the transformation of the camera with respect to the object (ie tobject^-1)
///
/// vobjectpoints is the output 3D points on the object coordinate system
void GetContactPointsFromImageRays(KinBodyPtr pbody, const vector<Vector>& vcamerapoints, const Transform& tcamera, vector<Vector>& vobjectpoints)
{
    CollisionReportPtr report(new CollisionReport());
    TransformMatrix tmcamera(tcamera);
    KinBody::KinBodyStateSaver saver(pbody);
    pbody->SetTransform(Transform());
    AABB ab = pbody->ComputeAABB();
    dReal maxdist = RaveFabs(tmcamera.m[2]*tmcamera.trans.x + tmcamera.m[6]*tmcamera.trans.y + tmcamera.m[10]*tmcamera.trans.z)+RaveSqrt(ab.extents.lengthsqr3())+0.2f;
    RAY r;
    for(size_t i = 0; i < vcamerapoints.size(); ++i) {
        r.dir = tcamera.rotate(vcamerapoints[i]*(maxdist/RaveSqrt(vcamerapoints[i].lengthsqr3())));
        r.pos = tcamera.trans;
        if( pbody->GetEnv()->CheckCollision(r,pbody,report) ) {
            vobjectpoints.push_back(report->contacts.at(0).pos);
        }
        else {
            vobjectpoints.push_back(Vector(1.0/0,1.0/0,1.0/0)); // nan
        }
    }
}

int main(int argc, char ** argv)
{
    EnvironmentBasePtr penv = CreateEnvironment(true);
    KinBodyPtr pbody = penv->ReadKinBodyXMLFile("brkt_hvac.kinbody.xml");
    penv->AddKinBody(pbody);
    vector<Vector> vcamerapoints, vobjectpoints;
    Transform tcamera;
    tcamera.trans.z = -1.8;
    vcamerapoints.push_back(Vector(0,0,1));
    GetContactPointsFromImageRays(pbody,vcamerapoints,tcamera,vobjectpoints);
    printf("%f %f %f\n",vobjectpoints[0].x,vobjectpoints[0].y,vobjectpoints[0].z);
    RaveDestroy();
    return 0;
}
