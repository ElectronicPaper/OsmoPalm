#include "control_math.h"
#include "haptic_envelope.h"
#include "motion_edit.h"
#include "motion_playback_ui.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
using namespace osmo;
using namespace osmo::control;

static bool near(float a, float b, float e=0.0001f) { return std::fabs(a-b) < e; }

int main(int argc, char** argv) {
    assert(argc == 2);
    const char* name = argv[1];
    if (!std::strcmp(name, "relative_pose")) {
        for (int axis=0; axis<3; ++axis) for (int degrees=-180; degrees<=180; degrees+=9) {
            const float half=degrees*0.0174532925f/2;
            Quat q{std::cos(half), axis==0 ? std::sin(half):0,
                   axis==1 ? std::sin(half):0, axis==2 ? std::sin(half):0};
            Vec3 delta=rotationVector(multiply(q, conjugate(q)));
            assert(norm(delta)<0.00001f);
            Vec3 v{.3f,-.7f,.1f};
            Vec3 back=rotateVector(conjugate(q),rotateVector(q,v));
            assert(near(back.x,v.x) && near(back.y,v.y) && near(back.z,v.z));
        }
        assert(HAND_TILT_BASE_SIGN == -1 && HAND_PAN_BASE_SIGN == -1);
    } else if (!std::strcmp(name, "shaping")) {
        for (int smooth=0;smooth<3;++smooth) for(int stability=0;stability<3;++stability) {
            AxisShaper shaper;
            float previous=0;
            for(int i=0;i<400;++i) {
                const float dt = i%3==0 ? .031f : .045f;
                float v=shaper.step(6,dt,smooth,stability);
                assert(v>=previous && v<=6 && std::isfinite(v));
                previous=v;
            }
            assert(near(previous,6));
            for(int i=0;i<400;++i) assert(shaper.step(-6,.04f,smooth,stability)>=-6);
            assert(near(shaper.value,-6));
            shaper.reset(); assert(shaper.value==0 && shaper.accel==0);
        }
    } else if (!std::strcmp(name, "precision")) {
        for (int stability=0;stability<3;++stability) {
            PrecisionIntentGate gate;
            for(int i=0;i<200;++i) assert(gate.step(.05f,.04f,true,stability)==0);
            float value=0;
            for(int i=0;i<200;++i) value=gate.step(2,.04f,true,stability);
            assert(near(value,2,.01f));
            assert(gate.step(0,.04f,true,stability)==0 && !gate.live);
            assert(gate.step(1,.04f,false,stability)==1);
        }
    } else if (!std::strcmp(name, "curve")) {
        for (int kind=0;kind<4;++kind) {
            const auto t=static_cast<MotionTransition>(kind);
            assert(near(sampleProgramCurve(t,0,3).position,0));
            assert(near(sampleProgramCurve(t,1,3).position,1));
            float last=0;
            for(int i=1;i<=100;++i) {
                auto v=sampleProgramCurve(t,i/100.f,3);
                assert(v.position>=last && v.position<=1 && v.rate>=0);
                last=v.position;
            }
        }
    } else if (!std::strcmp(name, "wire_limits")) {
        assert(stickAxis(0)==1024 && stickAxis(2)==1574 && stickAxis(-2)==474);
        assert(deflectionForRate(.34f)==0);
        assert(near(deflectionForRate(.35f),.1f));
        assert(near(deflectionForRate(-6),-deflectionForRate(6)));
        assert(limitWarning(0,0)==100);
        assert(limitWarning(50,0)==0);
        assert(near(wrap180(359),-1) && near(positive360(-1),359));
    } else if (!std::strcmp(name, "motion_delete")) {
        MotionProgram p; p.count=3;
        for(int i=0;i<3;++i) { p.points[i].captured=true; p.points[i].pitch=10*(i+1); p.points[i].moveMs=1000*(i+1); }
        assert(eraseMotionPoint(p,1));
        assert(p.count==2 && p.points[0].pitch==10 && p.points[1].pitch==30);
        assert(p.points[1].moveMs==3000 && !p.points[2].captured);
        assert(!eraseMotionPoint(p,2) && p.count==2);
        assert(eraseMotionPoint(p,0) && p.points[0].pitch==30);
        assert(eraseMotionPoint(p,0) && p.count==0);
        assert(!eraseMotionPoint(p,0));
    } else if (!std::strcmp(name, "soft_haptic")) {
        HapticEnvelope e;
        assert(e.schedule(1000,180,70,1,false));
        unsigned on=0;
        for(unsigned i=0;i<100;++i) {
            auto level=e.sample(1000+i);
            if(level) { ++on; assert(level==110); assert(480+12*level>=1800); }
        }
        assert(on>0 && on<=20);
    } else if (!std::strcmp(name, "haptic_profiles")) {
        for(unsigned profile=1;profile<=3;++profile) {
            HapticEnvelope e;
            assert(e.schedule(1000,255,65000,profile,false));
            unsigned on=0;
            for(unsigned i=0;i<200;++i) {
                auto level=e.sample(1000+i);
                if(level) {
                    ++on;
                    unsigned mv=480+12*level;
                    mv=1800+(mv-1800)/100*100;
                    assert(mv>=1800 && mv<=HapticEnvelope::profile(profile).peakMv);
                }
            }
            assert(on<=HapticEnvelope::profile(profile).maxOnMs);
        }
    } else if (!std::strcmp(name, "haptic_cancel")) {
        HapticEnvelope e;
        assert(e.schedule(1000,180,70,3,false));
        assert(e.sample(1005)>0);
        assert(!e.schedule(1006,180,70,0,false));
        assert(e.sample(1006)==0);
        assert(e.schedule(2000,180,70,3,false));
        assert(!e.schedule(2001,180,70,3,true));
        assert(e.sample(2001)==0);
    } else if (!std::strcmp(name, "haptic_deadline")) {
        HapticEnvelope e;
        assert(e.schedule(1000,180,70,1,false));
        assert(!e.schedule(1005,180,120,1,false));
        assert(e.sample(1020)==0);
        assert(!e.schedule(1100,180,70,1,false));
        assert(e.schedule(1140,180,70,1,false));
        assert(e.sample(1200)==0);
        HapticEnvelope wrap;
        const uint32_t start=std::numeric_limits<uint32_t>::max()-5;
        assert(wrap.schedule(start,180,70,1,false));
        assert(wrap.sample(start+10)==110);
        assert(wrap.sample(start+20)==0);
    } else if (!std::strcmp(name, "playback_actions")) {
        PlaybackContext c{true,true,false,false,false,false,2};
        assert(playbackAction(c)==PlaybackAction::GoToStart);
        c.atStart=true;
        assert(playbackAction(c)==PlaybackAction::Play);
        c.active=true;
        assert(playbackAction(c)==PlaybackAction::Stop);
        c.cue=true;
        assert(playbackAction(c)==PlaybackAction::Continue);
        c.fresh=false;
        assert(playbackAction(c)==PlaybackAction::Stop);
        c.batteryBlocked=true;
        assert(playbackAction(c)==PlaybackAction::Stop);
        c.active=false;
        assert(playbackAction(c)==PlaybackAction::None);
        c.fresh=true;
        assert(playbackAction(c)==PlaybackAction::None);
        c.batteryBlocked=false; c.points=1;
        assert(playbackAction(c)==PlaybackAction::None);
        c.points=2; c.direct=false;
        assert(playbackAction(c)==PlaybackAction::None);
    } else if (!std::strcmp(name, "manual_ease")) {
        for (int ms : {0,100,200,350,500,750,2000}) {
            for (float velocity : {-42.f,-3.f,0.f,3.f,42.f}) {
                float prior=std::fabs(velocity);
                for (unsigned t=0;t<=800;t+=10) {
                    float out=releaseEaseRate(velocity,t,ms);
                    assert(std::fabs(out)<=prior+0.0001f);
                    assert(out*velocity>=0);
                    prior=std::fabs(out);
                    if (t>=static_cast<unsigned>(std::min(ms,750))) assert(out==0);
                }
            }
        }
        for (int start : {100,200,350,500,750}) {
            AxisShaper shaper;
            float last=0;
            for(int i=0;i<250;++i) {
                float v=shaper.step(30,.04f,1,1,start,200,30);
                assert(v>=last-0.0001f && v<=30);
                last=v;
            }
            assert(near(last,30));
            shaper.reset();
            assert(shaper.value==0 && shaper.accel==0);
        }
    } else { return 2; }
    std::cout << name << " passed\n";
}
