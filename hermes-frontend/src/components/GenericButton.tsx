import { MouseEventHandler, PropsWithChildren, ReactNode } from "react";

export interface GenericButtonProps extends PropsWithChildren<any>{
    
}

export const GenericButton = ({className = "", children, href = undefined, onClick = () => {}}: GenericButtonProps) => href === undefined ? (
    <button 
        className={"border-solid border-4 transition duration-500 border-hermes-cyan-dark hover:border-black " + className}
        onClick={onClick}>
            {children}
    </button>
    ) :
    (<a href={href}>
        <button 
            className={"border-solid border-4 transition duration-500 border-hermes-cyan-dark hover:border-black " + className}
            onClick={onClick}>
                {children}
        </button>
    </a>);