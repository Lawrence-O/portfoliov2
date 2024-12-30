import {NavBar} from "@/app/components/shared/NavBar";
import {Footer} from "@/app/components/shared/Footer";
import { ReactNode } from "react";

export function Layout({children}: {children: ReactNode}) {
    return (
        <>
            <NavBar />
            {children}
            {/* <Footer /> */}
        </>
    )
}