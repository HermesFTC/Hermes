import { RootState, useAppDispatch } from "@/store/reducers";
import { CustomVarState, BasicVarState, ConfigVarState } from "@/store/types/config";
import { config, a, to } from "@react-spring/web";
import { useSelector } from "react-redux";

export function getVariableValue(path: string) {
    // get all states and map to config
    const configRoot = useSelector(
        (state: RootState) => state.config.configRoot,
    ) as CustomVarState;

    // get recursively
    let out = getVariableValueRecusively(path, configRoot)
    if (out == null) {
        console.warn("Path " + path + " is invalid (null).")
        return null;
    }
    return out;
}

function getVariableValueRecusively(path: string, state: ConfigVarState, iter: number = 1   ) {

    // return null
    if (state == null) {
        return null;
    }

    let paths = path.split("/")

    // return end of path
    if (paths.length == 1 && paths[0] == '') {
        return state;
    }

    // make sure current state is custom and it has children
    if (state.__type != 'custom' || state.__value == null) {
        return null;
    }

    // remove first index and save it
    let subpathString = paths[0]
    paths = paths.filter((value, index, array) => index != 0)

    // recreate path
    let newPath = ""
    for (let subpath in paths) {
        newPath += paths[subpath] + "/"
    }

    // remove ending slash
    newPath = newPath.slice(0, newPath.length - 1)

    return getVariableValueRecusively(newPath, state.__value[subpathString], iter + 1)


}

export function setVariableValue(path: string, value: boolean | number | string | null) {
    const dispatch = useAppDispatch();

    // get current config state
    const stateValue = getVariableValue(path);


    // check for null or custom
    if (stateValue == null) {
        console.warn("Path " + path + " is null.")
        return;
    }

    if (stateValue.__type == 'custom') {
        console.warn("Path " + path + " is not a variable, it is a larger group!")
        return;
    }

    stateValue.__value = value;
    stateValue.__newValue = value;

    // dispatch update with diff
    dispatch({
        type: 'SAVE_CONFIG',
        configDiff: constructConfigDiffRecursively(path, stateValue)
      }); 
}

/**
 * @param state The modified basic state.
 * @param path The path to the basic var, separated by /.
 */
function constructConfigDiffRecursively(path: string, state: BasicVarState): ConfigVarState {
    
    
    let paths = path.split("/")

    // base case: basic variables return
    if (paths.length == 1) {
        return state;
    }

    // remove first index and save it
    let subpathString = paths[0]
    paths = paths.filter((value, index, array) => index != 0)

    // recreate path
    let newPath = ""
    for (let subpath in paths) {
        newPath += subpath + "/"
    }

    // remove ending slash
    newPath = newPath.slice(0, newPath.length - 1)

    // return recursively
    return { __type: 'custom', __value: { subpathString: constructConfigDiffRecursively(newPath, state) } }

}