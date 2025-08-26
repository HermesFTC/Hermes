import { MouseEventHandler, PropsWithChildren, ReactNode } from "react";

interface GenericButtonProps extends PropsWithChildren<any>{
    
}

export const GenericButton = ({className = "", children, href = undefined, onClick = () => {}}: GenericButtonProps) => href === undefined ? (
    <button 
        className={"border-solid border-4 transition duration-500 border-hermes-cyan-dark hover:scale-125 " + className}
        onClick={onClick}>
            {children}
    </button>
    ) :
    (<a href={href}>
        <button 
            className={"border-solid border-4 transition duration-500 border-hermes-cyan-dark hover:scale-125 " + className}
            onClick={onClick}>
                {children}
        </button>
    </a>);