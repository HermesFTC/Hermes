import { useSelector } from "react-redux";
import { RootState, useAppDispatch } from "@/store/reducers"; // Assuming correct path to store
import { CustomVarState, BasicVarState, ConfigVarState } from "@/store/types/config";

/**
 * Fetch the variable value, through recursion
 */
function getVariableValueRecursively(path: string, state: ConfigVarState | null): ConfigVarState | null {
    if (state == null) {
        return null;
    }

    const paths = path.split("/");

    // base case: end of path
    if (paths.length === 1 && paths[0] === '') {
        return state;
    }

    // Ensure current state is custom and has children
    if (state.__type !== 'custom' || state.__value === null) {
        return null;
    }

    const subpathString = paths[0];
    const remainingPaths = paths.slice(1);
    const newPath = remainingPaths.join("/"); // Reconstruct path efficiently

    // recurse
    return getVariableValueRecursively(newPath, state.__value[subpathString] as ConfigVarState);
}


/**
 * Constructs a config diff object recursively.
 * @param path The path to the basic var, separated by /.
 * @param state The modified basic state.
 */
export function constructConfigDiffRecursively(path: string, state: BasicVarState): ConfigVarState {
    const paths = path.split("/");

    // Base case: go to end of path and return basic var
    if (paths.length === 1 && paths[0] == '') {
        return state;
    }

    const subpathString = paths[0];
    const remainingPaths = paths.slice(1);
    const newPath = remainingPaths.join("/");

    // Return recursively, building up the custom object structure
    return {
        __type: 'custom',
        __value: {
            [subpathString]: constructConfigDiffRecursively(newPath, state)
        }
    };
}

export function useConfigVariableState(path: string): ConfigVarState | null {
    // get all states and map to config
    const configRoot = useSelector(
        (state: RootState) => state.config.configRoot,
    ) as CustomVarState;

    // get recursively
    return getVariableValueRecursively(path, configRoot);
}

export function useConfigVariable(path: string): boolean | number | string | null {
    const out = useConfigVariableState(path);
    return out == null ? null : (out as BasicVarState).__value; // Cast to BasicVarState as CustomVarState won't have __value as primitive
}

export function useSetConfigVariable() {
    const dispatch = useAppDispatch();
    const configRoot = useSelector((state: RootState) => state.config.configRoot) as CustomVarState;

    const setVariable = (path: string, value: boolean | number | string | null) => {

        const stateValue = getVariableValueRecursively(path, configRoot);

        // check for null or custom
        if (stateValue == null) {
            console.warn("Path " + path + " is null.");
            return;
        }

        if (stateValue.__type === 'custom') {
            console.warn("Path " + path + " is not a variable, it is a larger group!");
            return;
        }

        // deep copy the state with updated values
        const modifiedStateValue: BasicVarState = {
            ...stateValue,
            __value: value,
            __newValue: value,
        };

        console.log(modifiedStateValue)


        // dispatch update with diff
        dispatch({
            type: 'SAVE_CONFIG',
            configDiff: constructConfigDiffRecursively(path, modifiedStateValue)
        });
    };

    return setVariable;
}
