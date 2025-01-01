import {NavBar} from "@/app/components/shared/NavBar";
import { ReactNode } from "react";

export default function Layout({children}: {children: ReactNode}) {
    return (
        <>
            <NavBar />
            {children}
        </>
    )
}