/**
  * @file <loop-functions/example/PwLoopFunc.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef HAB_HZ_LOOP_FUNC
#define HAB_HZ_LOOP_FUNC

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/phormica/simulator/block_entity.h>
#include <argos3/plugins/robots/phormica/simulator/phormica_entity.h>
#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class HabHzLoopFunction: public CoreLoopFunctions {
  public:
    HabHzLoopFunction();
    HabHzLoopFunction(const HabHzLoopFunction& orig);
    virtual ~HabHzLoopFunction();

    virtual void Destroy();

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();
    virtual void Init(TConfigurationNode& t_tree);

    Real GetObjectiveFunction();

    CVector3 GetRandomPosition();
    UInt32 GetRandomTime(UInt32 unMin, UInt32 unMax);

    void InitRobotStates();
    void UpdateRobotPositions();

    void InitPhormicaState();
    void UpdatePhormicaState();

    void InitMocaState();
    void MocaControl();

    void ScoreControl();
    Real GetRobotInScore();
    Real GetRobotOutScore();

  private:

    /*
     * Method used to deternmine wheter a number is even.
     */
    bool IsEven(UInt32 unNumber);

    /*
     * Return the radious of the arena.
     */
    Real GetArenaRadious();

    /*
     * The arena used in the experiment.
     */
    CPhormicaEntity* m_pcPhormica;

    /*
     * Transition time in sequence experiments
     */
    UInt32 m_unTrnTime;

    /*
     * Allows for score invertion in maximization algorithms.
     */
    bool m_bMaximization;

    UInt32 m_unClock;
    UInt32 m_unStopTime;
    UInt32 m_unStopBlock;
    UInt32 m_unNumberLEDs;
    Real m_fObjectiveFunction;

    /*
    * Parameter to determine the width of pheromone trail based on the UV LEDs
    */

    Real m_fPheromoneParameter;

    struct RobotStateStruct {
        CVector2 cLastPosition;
        CVector2 cPosition;
        UInt32 unItem;
        UInt32 unPheromoneLEDs;
    };

    struct LEDStateStruct {
        CVector2 cLEDPosition;
        UInt32 unLEDIndex;
        UInt32 unTimer;
    };

    typedef std::map<CEPuckEntity*, RobotStateStruct> TRobotStateMap;

    typedef std::map<UInt32, LEDStateStruct> TLEDStateMap;

    TRobotStateMap m_tRobotStates;
    TLEDStateMap m_tLEDStates;

    CColor m_cUVColor;

};

#endif
