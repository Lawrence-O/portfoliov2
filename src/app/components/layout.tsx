import { ReactNode } from "react";

import {NavBar} from "@/app/components/shared/NavBar";

export default function Layout({children}: {children: ReactNode}) {
    return (
        <>
            <NavBar />
            {children}
        </>
    )
}