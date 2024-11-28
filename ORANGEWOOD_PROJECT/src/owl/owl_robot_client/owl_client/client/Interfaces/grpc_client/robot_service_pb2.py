# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: robot_service.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13robot_service.proto\x12\rrobot_service\"T\n\nMultiArray\x12/\n\x06\x61rrays\x18\x01 \x03(\x0b\x32\x1f.robot_service.MultiArray.Array\x1a\x15\n\x05\x41rray\x12\x0c\n\x04\x64\x61ta\x18\x01 \x03(\x01\"\r\n\x0bPingRequest\"\x0e\n\x0cPingResponse\"%\n\x15SetPayloadMassRequest\x12\x0c\n\x04mass\x18\x01 \x01(\x01\"*\n\x16SetPayloadMassResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\'\n\x18SetPayloadCOGLoadRequest\x12\x0b\n\x03\x63og\x18\x01 \x03(\x01\")\n\x15SetPayloadCOGResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"1\n\x1eSetPayloadInertiaMatrixRequest\x12\x0f\n\x07inertia\x18\x01 \x03(\x01\"3\n\x1fSetPayloadInertiaMatrixResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"?\n\x11SetPayloadRequest\x12\x0c\n\x04mass\x18\x01 \x01(\x01\x12\x0b\n\x03\x63og\x18\x02 \x03(\x01\x12\x0f\n\x07inertia\x18\x03 \x03(\x01\"&\n\x12SetPayloadResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\"\n\rSetTCPRequest\x12\x11\n\ttransform\x18\x01 \x03(\x01\"\"\n\x0eSetTCPResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\")\n\x17SetGravityVectorRequest\x12\x0e\n\x06vector\x18\x01 \x03(\x01\",\n\x18SetGravityVectorResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"/\n\x1fSetCartesianJoggingFrameRequest\x12\x0c\n\x04type\x18\x01 \x01(\x05\"4\n SetCartesianJoggingFrameResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"A\n\x17SetDigitalOutputRequest\x12\x12\n\noutput_pin\x18\x01 \x01(\x05\x12\x12\n\npin_status\x18\x02 \x01(\x08\",\n\x18SetDigitalOutputResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"9\n\x18SetDefaultProgramRequest\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0f\n\x07program\x18\x02 \x01(\t\"-\n\x19SetDefaultProgramResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"1\n\x17SetSpeedFractionRequest\x12\x16\n\x0espeed_fraction\x18\x01 \x01(\x01\",\n\x18SetSpeedFractionResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"7\n\x1eSetDefaultConfigurationRequest\x12\x15\n\rconfiguration\x18\x01 \x01(\t\"3\n\x1fSetDefaultConfigurationResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"&\n\x15SetDefaultHomeRequest\x12\r\n\x05joint\x18\x01 \x03(\x01\"*\n\x16SetDefaultHomeResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"6\n\x1dSetActiveConfigurationRequest\x12\x15\n\rconfiguration\x18\x01 \x01(\t\"2\n\x1eSetActiveConfigurationResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x1f\n\x1dGetActiveConfigurationRequest\"7\n\x1eGetActiveConfigurationResponse\x12\x15\n\rconfiguration\x18\x01 \x01(\t\".\n\x1eGetPayloadInertiaMatrixRequest\x12\x0c\n\x04mass\x18\x01 \x01(\x01\"2\n\x1fGetPayloadInertiaMatrixResponse\x12\x0f\n\x07inertia\x18\x01 \x03(\x01\" \n\x1eGetDefaultConfigurationRequest\"8\n\x1fGetDefaultConfigurationResponse\x12\x15\n\rconfiguration\x18\x01 \x01(\t\"\x13\n\x11GetVersionRequest\"%\n\x12GetVersionResponse\x12\x0f\n\x07version\x18\x01 \x01(\t\"\x10\n\x0ePowerOnRequest\"#\n\x0fPowerOnResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x12\n\x10PowerDownRequest\"%\n\x11PowerDownResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x17\n\x15\x45nterTeachModeRequest\"*\n\x16\x45nterTeachModeResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x16\n\x14\x45xitTeachModeRequest\")\n\x15\x45xitTeachModeResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\"\n\x10\x46KRequestRequest\x12\x0e\n\x06joints\x18\x01 \x03(\x01\"!\n\x11\x46KRequestResponse\x12\x0c\n\x04pose\x18\x01 \x03(\x01\"7\n\x10IKRequestRequest\x12\x15\n\rinitial_guess\x18\x01 \x03(\x01\x12\x0c\n\x04pose\x18\x02 \x03(\x01\"#\n\x11IKRequestResponse\x12\x0e\n\x06joints\x18\x01 \x03(\x01\"$\n\x11SendScriptRequest\x12\x0f\n\x07program\x18\x01 \x01(\t\"&\n\x12SendScriptResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\";\n\x1dSendOperatorAssignmentRequest\x12\x1a\n\x12operatorAssignment\x18\x01 \x01(\t\"2\n\x1eSendOperatorAssignmentResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x14\n\x12PauseScriptRequest\"\'\n\x13PauseScriptResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x15\n\x13ResumeScriptRequest\"(\n\x14ResumeScriptResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x13\n\x11StopScriptRequest\"&\n\x12StopScriptResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"V\n\x11MoveToPoseRequest\x12\x0c\n\x04pose\x18\x01 \x03(\x01\x12\x12\n\ntool_speed\x18\x02 \x01(\x01\x12\x11\n\tmove_type\x18\x03 \x01(\x05\x12\x0c\n\x04wait\x18\x04 \x01(\x08\"&\n\x12MoveToPoseResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"G\n\x12MoveToJointRequest\x12\x0e\n\x06joints\x18\x01 \x03(\x01\x12\x13\n\x0bjoint_speed\x18\x02 \x01(\x01\x12\x0c\n\x04wait\x18\x03 \x01(\x08\"\'\n\x13MoveToJointResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"^\n\x10MovePosesRequest\x12\x12\n\nposes_data\x18\x01 \x01(\x0c\x12\x12\n\ntool_speed\x18\x02 \x01(\x01\x12\x14\n\x0c\x62lend_factor\x18\x03 \x01(\x01\x12\x0c\n\x04wait\x18\x04 \x01(\x08\"%\n\x11MovePosesResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"c\n\x15MoveTrajectoryRequest\x12\x14\n\x0cjoint_points\x18\x01 \x01(\x0c\x12\x17\n\x0fvelocity_points\x18\x02 \x01(\x0c\x12\x1b\n\x13\x61\x63\x63\x65leration_points\x18\x03 \x01(\x0c\"*\n\x16MoveTrajectoryResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x12\n\x10MovePauseRequest\"%\n\x11MovePauseResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x13\n\x11MoveResumeRequest\"&\n\x12MoveResumeResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x12\n\x10MoveAbortRequest\"%\n\x11MoveAbortResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x12\n\x10\x43onfigureRequest\"%\n\x11\x43onfigureResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x19\n\x17\x44oneCallibrationRequest\",\n\x18\x44oneCallibrationResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x1a\n\x18ResetCallibrationRequest\"-\n\x19ResetCallibrationResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\")\n\x13SystemUpdateRequest\x12\x12\n\naccessCode\x18\x01 \x01(\t\"(\n\x14SystemUpdateResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\")\n\x13SystemEnableRequest\x12\x12\n\naccessCode\x18\x01 \x01(\t\"(\n\x14SystemEnableResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"*\n\x14SystemDisableRequest\x12\x12\n\naccessCode\x18\x01 \x01(\t\")\n\x15SystemDisableResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\"\x15\n\x13SystemRebootRequest\"\x16\n\x14SystemRebootResponse\"\x17\n\x15SystemShutdownRequest\"*\n\x16SystemShutdownResponse\x12\x10\n\x08response\x18\x01 \x01(\x08\x32\x8b!\n\x0cRobotService\x12\x41\n\x04Ping\x12\x1a.robot_service.PingRequest\x1a\x1b.robot_service.PingResponse\"\x00\x12_\n\x0eSetPayloadMass\x12$.robot_service.SetPayloadMassRequest\x1a%.robot_service.SetPayloadMassResponse\"\x00\x12`\n\rSetPayloadCOG\x12\'.robot_service.SetPayloadCOGLoadRequest\x1a$.robot_service.SetPayloadCOGResponse\"\x00\x12z\n\x17SetPayloadInertiaMatrix\x12-.robot_service.SetPayloadInertiaMatrixRequest\x1a..robot_service.SetPayloadInertiaMatrixResponse\"\x00\x12S\n\nSetPayload\x12 .robot_service.SetPayloadRequest\x1a!.robot_service.SetPayloadResponse\"\x00\x12G\n\x06SetTCP\x12\x1c.robot_service.SetTCPRequest\x1a\x1d.robot_service.SetTCPResponse\"\x00\x12\x65\n\x10SetGravityVector\x12&.robot_service.SetGravityVectorRequest\x1a\'.robot_service.SetGravityVectorResponse\"\x00\x12}\n\x18SetCartesianJoggingFrame\x12..robot_service.SetCartesianJoggingFrameRequest\x1a/.robot_service.SetCartesianJoggingFrameResponse\"\x00\x12\x65\n\x10SetDigitalOutput\x12&.robot_service.SetDigitalOutputRequest\x1a\'.robot_service.SetDigitalOutputResponse\"\x00\x12h\n\x11SetDefaultProgram\x12\'.robot_service.SetDefaultProgramRequest\x1a(.robot_service.SetDefaultProgramResponse\"\x00\x12_\n\x0eSetDefaultHome\x12$.robot_service.SetDefaultHomeRequest\x1a%.robot_service.SetDefaultHomeResponse\"\x00\x12\x65\n\x10SetSpeedFraction\x12&.robot_service.SetSpeedFractionRequest\x1a\'.robot_service.SetSpeedFractionResponse\"\x00\x12z\n\x17SetDefaultConfiguration\x12-.robot_service.SetDefaultConfigurationRequest\x1a..robot_service.SetDefaultConfigurationResponse\"\x00\x12w\n\x16SetActiveConfiguration\x12,.robot_service.SetActiveConfigurationRequest\x1a-.robot_service.SetActiveConfigurationResponse\"\x00\x12z\n\x17GetPayloadInertiaMatrix\x12-.robot_service.GetPayloadInertiaMatrixRequest\x1a..robot_service.GetPayloadInertiaMatrixResponse\"\x00\x12w\n\x16GetActiveConfiguration\x12,.robot_service.GetActiveConfigurationRequest\x1a-.robot_service.GetActiveConfigurationResponse\"\x00\x12z\n\x17GetDefaultConfiguration\x12-.robot_service.GetDefaultConfigurationRequest\x1a..robot_service.GetDefaultConfigurationResponse\"\x00\x12S\n\nGetVersion\x12 .robot_service.GetVersionRequest\x1a!.robot_service.GetVersionResponse\"\x00\x12J\n\x07PowerOn\x12\x1d.robot_service.PowerOnRequest\x1a\x1e.robot_service.PowerOnResponse\"\x00\x12P\n\tPowerDown\x12\x1f.robot_service.PowerDownRequest\x1a .robot_service.PowerDownResponse\"\x00\x12_\n\x0e\x45nterTeachMode\x12$.robot_service.EnterTeachModeRequest\x1a%.robot_service.EnterTeachModeResponse\"\x00\x12\\\n\rExitTeachMode\x12#.robot_service.ExitTeachModeRequest\x1a$.robot_service.ExitTeachModeResponse\"\x00\x12P\n\tFKRequest\x12\x1f.robot_service.FKRequestRequest\x1a .robot_service.FKRequestResponse\"\x00\x12P\n\tIKRequest\x12\x1f.robot_service.IKRequestRequest\x1a .robot_service.IKRequestResponse\"\x00\x12S\n\nSendScript\x12 .robot_service.SendScriptRequest\x1a!.robot_service.SendScriptResponse\"\x00\x12w\n\x16SendOperatorAssignment\x12,.robot_service.SendOperatorAssignmentRequest\x1a-.robot_service.SendOperatorAssignmentResponse\"\x00\x12V\n\x0bPauseScript\x12!.robot_service.PauseScriptRequest\x1a\".robot_service.PauseScriptResponse\"\x00\x12Y\n\x0cResumeScript\x12\".robot_service.ResumeScriptRequest\x1a#.robot_service.ResumeScriptResponse\"\x00\x12S\n\nStopScript\x12 .robot_service.StopScriptRequest\x1a!.robot_service.StopScriptResponse\"\x00\x12S\n\nMoveToPose\x12 .robot_service.MoveToPoseRequest\x1a!.robot_service.MoveToPoseResponse\"\x00\x12V\n\x0bMoveToJoint\x12!.robot_service.MoveToJointRequest\x1a\".robot_service.MoveToJointResponse\"\x00\x12P\n\tMovePoses\x12\x1f.robot_service.MovePosesRequest\x1a .robot_service.MovePosesResponse\"\x00\x12_\n\x0eMoveTrajectory\x12$.robot_service.MoveTrajectoryRequest\x1a%.robot_service.MoveTrajectoryResponse\"\x00\x12P\n\tMovePause\x12\x1f.robot_service.MovePauseRequest\x1a .robot_service.MovePauseResponse\"\x00\x12S\n\nMoveResume\x12 .robot_service.MoveResumeRequest\x1a!.robot_service.MoveResumeResponse\"\x00\x12P\n\tMoveAbort\x12\x1f.robot_service.MoveAbortRequest\x1a .robot_service.MoveAbortResponse\"\x00\x12P\n\tConfigure\x12\x1f.robot_service.ConfigureRequest\x1a .robot_service.ConfigureResponse\"\x00\x12\x65\n\x10\x44oneCallibration\x12&.robot_service.DoneCallibrationRequest\x1a\'.robot_service.DoneCallibrationResponse\"\x00\x12h\n\x11ResetCallibration\x12\'.robot_service.ResetCallibrationRequest\x1a(.robot_service.ResetCallibrationResponse\"\x00\x12Y\n\x0cSystemUpdate\x12\".robot_service.SystemUpdateRequest\x1a#.robot_service.SystemUpdateResponse\"\x00\x12Y\n\x0cSystemEnable\x12\".robot_service.SystemEnableRequest\x1a#.robot_service.SystemEnableResponse\"\x00\x12\\\n\rSystemDisable\x12#.robot_service.SystemDisableRequest\x1a$.robot_service.SystemDisableResponse\"\x00\x12Y\n\x0cSystemReboot\x12\".robot_service.SystemRebootRequest\x1a#.robot_service.SystemRebootResponse\"\x00\x12_\n\x0eSystemShutdown\x12$.robot_service.SystemShutdownRequest\x1a%.robot_service.SystemShutdownResponse\"\x00\x62\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'robot_service_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _MULTIARRAY._serialized_start=38
  _MULTIARRAY._serialized_end=122
  _MULTIARRAY_ARRAY._serialized_start=101
  _MULTIARRAY_ARRAY._serialized_end=122
  _PINGREQUEST._serialized_start=124
  _PINGREQUEST._serialized_end=137
  _PINGRESPONSE._serialized_start=139
  _PINGRESPONSE._serialized_end=153
  _SETPAYLOADMASSREQUEST._serialized_start=155
  _SETPAYLOADMASSREQUEST._serialized_end=192
  _SETPAYLOADMASSRESPONSE._serialized_start=194
  _SETPAYLOADMASSRESPONSE._serialized_end=236
  _SETPAYLOADCOGLOADREQUEST._serialized_start=238
  _SETPAYLOADCOGLOADREQUEST._serialized_end=277
  _SETPAYLOADCOGRESPONSE._serialized_start=279
  _SETPAYLOADCOGRESPONSE._serialized_end=320
  _SETPAYLOADINERTIAMATRIXREQUEST._serialized_start=322
  _SETPAYLOADINERTIAMATRIXREQUEST._serialized_end=371
  _SETPAYLOADINERTIAMATRIXRESPONSE._serialized_start=373
  _SETPAYLOADINERTIAMATRIXRESPONSE._serialized_end=424
  _SETPAYLOADREQUEST._serialized_start=426
  _SETPAYLOADREQUEST._serialized_end=489
  _SETPAYLOADRESPONSE._serialized_start=491
  _SETPAYLOADRESPONSE._serialized_end=529
  _SETTCPREQUEST._serialized_start=531
  _SETTCPREQUEST._serialized_end=565
  _SETTCPRESPONSE._serialized_start=567
  _SETTCPRESPONSE._serialized_end=601
  _SETGRAVITYVECTORREQUEST._serialized_start=603
  _SETGRAVITYVECTORREQUEST._serialized_end=644
  _SETGRAVITYVECTORRESPONSE._serialized_start=646
  _SETGRAVITYVECTORRESPONSE._serialized_end=690
  _SETCARTESIANJOGGINGFRAMEREQUEST._serialized_start=692
  _SETCARTESIANJOGGINGFRAMEREQUEST._serialized_end=739
  _SETCARTESIANJOGGINGFRAMERESPONSE._serialized_start=741
  _SETCARTESIANJOGGINGFRAMERESPONSE._serialized_end=793
  _SETDIGITALOUTPUTREQUEST._serialized_start=795
  _SETDIGITALOUTPUTREQUEST._serialized_end=860
  _SETDIGITALOUTPUTRESPONSE._serialized_start=862
  _SETDIGITALOUTPUTRESPONSE._serialized_end=906
  _SETDEFAULTPROGRAMREQUEST._serialized_start=908
  _SETDEFAULTPROGRAMREQUEST._serialized_end=965
  _SETDEFAULTPROGRAMRESPONSE._serialized_start=967
  _SETDEFAULTPROGRAMRESPONSE._serialized_end=1012
  _SETSPEEDFRACTIONREQUEST._serialized_start=1014
  _SETSPEEDFRACTIONREQUEST._serialized_end=1063
  _SETSPEEDFRACTIONRESPONSE._serialized_start=1065
  _SETSPEEDFRACTIONRESPONSE._serialized_end=1109
  _SETDEFAULTCONFIGURATIONREQUEST._serialized_start=1111
  _SETDEFAULTCONFIGURATIONREQUEST._serialized_end=1166
  _SETDEFAULTCONFIGURATIONRESPONSE._serialized_start=1168
  _SETDEFAULTCONFIGURATIONRESPONSE._serialized_end=1219
  _SETDEFAULTHOMEREQUEST._serialized_start=1221
  _SETDEFAULTHOMEREQUEST._serialized_end=1259
  _SETDEFAULTHOMERESPONSE._serialized_start=1261
  _SETDEFAULTHOMERESPONSE._serialized_end=1303
  _SETACTIVECONFIGURATIONREQUEST._serialized_start=1305
  _SETACTIVECONFIGURATIONREQUEST._serialized_end=1359
  _SETACTIVECONFIGURATIONRESPONSE._serialized_start=1361
  _SETACTIVECONFIGURATIONRESPONSE._serialized_end=1411
  _GETACTIVECONFIGURATIONREQUEST._serialized_start=1413
  _GETACTIVECONFIGURATIONREQUEST._serialized_end=1444
  _GETACTIVECONFIGURATIONRESPONSE._serialized_start=1446
  _GETACTIVECONFIGURATIONRESPONSE._serialized_end=1501
  _GETPAYLOADINERTIAMATRIXREQUEST._serialized_start=1503
  _GETPAYLOADINERTIAMATRIXREQUEST._serialized_end=1549
  _GETPAYLOADINERTIAMATRIXRESPONSE._serialized_start=1551
  _GETPAYLOADINERTIAMATRIXRESPONSE._serialized_end=1601
  _GETDEFAULTCONFIGURATIONREQUEST._serialized_start=1603
  _GETDEFAULTCONFIGURATIONREQUEST._serialized_end=1635
  _GETDEFAULTCONFIGURATIONRESPONSE._serialized_start=1637
  _GETDEFAULTCONFIGURATIONRESPONSE._serialized_end=1693
  _GETVERSIONREQUEST._serialized_start=1695
  _GETVERSIONREQUEST._serialized_end=1714
  _GETVERSIONRESPONSE._serialized_start=1716
  _GETVERSIONRESPONSE._serialized_end=1753
  _POWERONREQUEST._serialized_start=1755
  _POWERONREQUEST._serialized_end=1771
  _POWERONRESPONSE._serialized_start=1773
  _POWERONRESPONSE._serialized_end=1808
  _POWERDOWNREQUEST._serialized_start=1810
  _POWERDOWNREQUEST._serialized_end=1828
  _POWERDOWNRESPONSE._serialized_start=1830
  _POWERDOWNRESPONSE._serialized_end=1867
  _ENTERTEACHMODEREQUEST._serialized_start=1869
  _ENTERTEACHMODEREQUEST._serialized_end=1892
  _ENTERTEACHMODERESPONSE._serialized_start=1894
  _ENTERTEACHMODERESPONSE._serialized_end=1936
  _EXITTEACHMODEREQUEST._serialized_start=1938
  _EXITTEACHMODEREQUEST._serialized_end=1960
  _EXITTEACHMODERESPONSE._serialized_start=1962
  _EXITTEACHMODERESPONSE._serialized_end=2003
  _FKREQUESTREQUEST._serialized_start=2005
  _FKREQUESTREQUEST._serialized_end=2039
  _FKREQUESTRESPONSE._serialized_start=2041
  _FKREQUESTRESPONSE._serialized_end=2074
  _IKREQUESTREQUEST._serialized_start=2076
  _IKREQUESTREQUEST._serialized_end=2131
  _IKREQUESTRESPONSE._serialized_start=2133
  _IKREQUESTRESPONSE._serialized_end=2168
  _SENDSCRIPTREQUEST._serialized_start=2170
  _SENDSCRIPTREQUEST._serialized_end=2206
  _SENDSCRIPTRESPONSE._serialized_start=2208
  _SENDSCRIPTRESPONSE._serialized_end=2246
  _SENDOPERATORASSIGNMENTREQUEST._serialized_start=2248
  _SENDOPERATORASSIGNMENTREQUEST._serialized_end=2307
  _SENDOPERATORASSIGNMENTRESPONSE._serialized_start=2309
  _SENDOPERATORASSIGNMENTRESPONSE._serialized_end=2359
  _PAUSESCRIPTREQUEST._serialized_start=2361
  _PAUSESCRIPTREQUEST._serialized_end=2381
  _PAUSESCRIPTRESPONSE._serialized_start=2383
  _PAUSESCRIPTRESPONSE._serialized_end=2422
  _RESUMESCRIPTREQUEST._serialized_start=2424
  _RESUMESCRIPTREQUEST._serialized_end=2445
  _RESUMESCRIPTRESPONSE._serialized_start=2447
  _RESUMESCRIPTRESPONSE._serialized_end=2487
  _STOPSCRIPTREQUEST._serialized_start=2489
  _STOPSCRIPTREQUEST._serialized_end=2508
  _STOPSCRIPTRESPONSE._serialized_start=2510
  _STOPSCRIPTRESPONSE._serialized_end=2548
  _MOVETOPOSEREQUEST._serialized_start=2550
  _MOVETOPOSEREQUEST._serialized_end=2636
  _MOVETOPOSERESPONSE._serialized_start=2638
  _MOVETOPOSERESPONSE._serialized_end=2676
  _MOVETOJOINTREQUEST._serialized_start=2678
  _MOVETOJOINTREQUEST._serialized_end=2749
  _MOVETOJOINTRESPONSE._serialized_start=2751
  _MOVETOJOINTRESPONSE._serialized_end=2790
  _MOVEPOSESREQUEST._serialized_start=2792
  _MOVEPOSESREQUEST._serialized_end=2886
  _MOVEPOSESRESPONSE._serialized_start=2888
  _MOVEPOSESRESPONSE._serialized_end=2925
  _MOVETRAJECTORYREQUEST._serialized_start=2927
  _MOVETRAJECTORYREQUEST._serialized_end=3026
  _MOVETRAJECTORYRESPONSE._serialized_start=3028
  _MOVETRAJECTORYRESPONSE._serialized_end=3070
  _MOVEPAUSEREQUEST._serialized_start=3072
  _MOVEPAUSEREQUEST._serialized_end=3090
  _MOVEPAUSERESPONSE._serialized_start=3092
  _MOVEPAUSERESPONSE._serialized_end=3129
  _MOVERESUMEREQUEST._serialized_start=3131
  _MOVERESUMEREQUEST._serialized_end=3150
  _MOVERESUMERESPONSE._serialized_start=3152
  _MOVERESUMERESPONSE._serialized_end=3190
  _MOVEABORTREQUEST._serialized_start=3192
  _MOVEABORTREQUEST._serialized_end=3210
  _MOVEABORTRESPONSE._serialized_start=3212
  _MOVEABORTRESPONSE._serialized_end=3249
  _CONFIGUREREQUEST._serialized_start=3251
  _CONFIGUREREQUEST._serialized_end=3269
  _CONFIGURERESPONSE._serialized_start=3271
  _CONFIGURERESPONSE._serialized_end=3308
  _DONECALLIBRATIONREQUEST._serialized_start=3310
  _DONECALLIBRATIONREQUEST._serialized_end=3335
  _DONECALLIBRATIONRESPONSE._serialized_start=3337
  _DONECALLIBRATIONRESPONSE._serialized_end=3381
  _RESETCALLIBRATIONREQUEST._serialized_start=3383
  _RESETCALLIBRATIONREQUEST._serialized_end=3409
  _RESETCALLIBRATIONRESPONSE._serialized_start=3411
  _RESETCALLIBRATIONRESPONSE._serialized_end=3456
  _SYSTEMUPDATEREQUEST._serialized_start=3458
  _SYSTEMUPDATEREQUEST._serialized_end=3499
  _SYSTEMUPDATERESPONSE._serialized_start=3501
  _SYSTEMUPDATERESPONSE._serialized_end=3541
  _SYSTEMENABLEREQUEST._serialized_start=3543
  _SYSTEMENABLEREQUEST._serialized_end=3584
  _SYSTEMENABLERESPONSE._serialized_start=3586
  _SYSTEMENABLERESPONSE._serialized_end=3626
  _SYSTEMDISABLEREQUEST._serialized_start=3628
  _SYSTEMDISABLEREQUEST._serialized_end=3670
  _SYSTEMDISABLERESPONSE._serialized_start=3672
  _SYSTEMDISABLERESPONSE._serialized_end=3713
  _SYSTEMREBOOTREQUEST._serialized_start=3715
  _SYSTEMREBOOTREQUEST._serialized_end=3736
  _SYSTEMREBOOTRESPONSE._serialized_start=3738
  _SYSTEMREBOOTRESPONSE._serialized_end=3760
  _SYSTEMSHUTDOWNREQUEST._serialized_start=3762
  _SYSTEMSHUTDOWNREQUEST._serialized_end=3785
  _SYSTEMSHUTDOWNRESPONSE._serialized_start=3787
  _SYSTEMSHUTDOWNRESPONSE._serialized_end=3829
  _ROBOTSERVICE._serialized_start=3832
  _ROBOTSERVICE._serialized_end=8067
# @@protoc_insertion_point(module_scope)