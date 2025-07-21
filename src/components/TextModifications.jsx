import { ReactNode } from 'react'

export function ImportantInstruction({children}) { 
    return <b className="text-red-900">{children}</b> 
}