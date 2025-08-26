import { useSelector, useDispatch } from 'react-redux';
import { RootState } from '@/store/reducers';
import { initOpMode, startOpMode, stopOpMode } from '@/store/actions/opmode';
import OpModeStatus from '@/enums/OpModeStatus';
import { STOP_OP_MODE_TAG } from '@/store/types/opmode';

export interface UseOpModeReturn {
  // State
  available: boolean;
  activeOpMode: string;
  activeOpModeStatus: typeof OpModeStatus[keyof typeof OpModeStatus];
  opModeList: string[];
  warningMessage: string;
  errorMessage: string;
  
  // Actions
  initOpMode: (opModeName: string) => void;
  startOpMode: () => void;
  stopOpMode: () => void;
  
  // Helper functions
  isOpModeRunning: boolean;
  isOpModeInitialized: boolean;
  isOpModeStopped: boolean;
  canInitOpMode: boolean;
  canStartOpMode: boolean;
  canStopOpMode: boolean;
}

/**
 * Custom hook for managing op modes
 * Provides state and actions for initializing, starting, and stopping op modes
 */
export function useOpMode(): UseOpModeReturn {
  const dispatch = useDispatch();
  
  // Get status state from Redux
  const {
    available,
    activeOpMode,
    activeOpModeStatus,
    opModeList,
    warningMessage,
    errorMessage,
  } = useSelector((state: RootState) => state.status);

  // Action creators
  const handleInitOpMode = (opModeName: string) => {
    dispatch(initOpMode(opModeName));
  };

  const handleStartOpMode = () => {
    dispatch(startOpMode());
  };

  const handleStopOpMode = () => {
    dispatch(stopOpMode());
  };

  // Helper computed values
  const isOpModeRunning = activeOpModeStatus === OpModeStatus.RUNNING;
  const isOpModeInitialized = activeOpModeStatus === OpModeStatus.INIT;
  const isOpModeStopped = activeOpMode === STOP_OP_MODE_TAG;
  
  // Conditions for when actions are allowed
  const canInitOpMode = available && opModeList.length > 0 && isOpModeStopped;
  const canStartOpMode = available && isOpModeInitialized;
  const canStopOpMode = available && (isOpModeInitialized || isOpModeRunning);

  return {
    // State
    available,
    activeOpMode,
    activeOpModeStatus,
    opModeList,
    warningMessage,
    errorMessage,
    
    // Actions
    initOpMode: handleInitOpMode,
    startOpMode: handleStartOpMode,
    stopOpMode: handleStopOpMode,
    
    // Helper values
    isOpModeRunning,
    isOpModeInitialized,
    isOpModeStopped,
    canInitOpMode,
    canStartOpMode,
    canStopOpMode,
  };
} 