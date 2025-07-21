import { getLayoutPreset } from "@/store/actions/settings";
import { startSocketWatcher } from "@/store/middleware/socketMiddleware";
import { useEffect } from "react";
import { useDispatch } from "react-redux/es/hooks/useDispatch";
import { Outlet } from "react-router-dom";

export default function DefaultLayout() {
    // start dash websocket
    const dispatch = useDispatch();

    useEffect(() => {
        dispatch(getLayoutPreset());

        startSocketWatcher(dispatch);
    }, [dispatch]);

    return (
        <Outlet/>
    )
}