set PLATFORM_NAME zu_onfi8
set SYS_PROJ_NAME nvme_ftl_system
set FTL_PROJ_NAME nvme_ftl
set FIL_PROJ_NAME nvme_fil
set ECC_PROJ_NAME nvme_ecc

set XSA_FILE /home/jimx/myssd/vivado_project/system_wrapper.xsa

catch {platform remove $PLATFORM_NAME}
catch {app remove $FTL_PROJ_NAME}
catch {app remove $FIL_PROJ_NAME}
catch {app remove $ECC_PROJ_NAME}

platform create -name $PLATFORM_NAME \
-hw $XSA_FILE \
-proc {psu_cortexa53_0} -os {standalone} -arch {64-bit} -fsbl-target {psu_cortexa53_0}

platform write
platform generate -domains
platform active {zu_onfi8}
bsp reload
bsp setlib -name xilffs -ver 4.4
bsp write
bsp reload
catch {bsp regenerate}
platform active {zu_onfi8}
domain create -name {standalone_psu_cortexr5_0} -display-name {standalone_psu_cortexr5_0} -os {standalone} -proc {psu_cortexr5_0} -runtime {cpp} -arch {32-bit} -support-app {empty_application}
domain create -name {standalone_psu_cortexr5_1} -display-name {standalone_psu_cortexr5_1} -os {standalone} -proc {psu_cortexr5_1} -runtime {cpp} -arch {32-bit} -support-app {empty_application}
platform generate -domains
platform write
domain active {zynqmp_fsbl}
domain active {zynqmp_pmufw}
domain active {standalone_domain}
domain active {standalone_psu_cortexr5_0}
domain active {standalone_psu_cortexr5_1}
platform generate -quick
platform generate

# Create FTL project
app create -name $FTL_PROJ_NAME -sysproj $SYS_PROJ_NAME -platform $PLATFORM_NAME -domain standalone_domain -template "Empty Application"
app config -name $FTL_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/src}"
app config -name $FTL_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/include}"
app config -name $FTL_PROJ_NAME -add libraries m

# Create FIL project
app create -name $FIL_PROJ_NAME -sysproj $SYS_PROJ_NAME -platform $PLATFORM_NAME -domain standalone_psu_cortexr5_0 -template "Empty Application"
app config -name $FIL_PROJ_NAME -add include-path "\${workspace_loc:/$FIL_PROJ_NAME/src}"
app config -name $FIL_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/include}"

# Create ECC project
app create -name $ECC_PROJ_NAME -sysproj $SYS_PROJ_NAME -platform $PLATFORM_NAME -domain standalone_psu_cortexr5_1 -template "Empty Application"
app config -name $ECC_PROJ_NAME -add include-path "\${workspace_loc:/$ECC_PROJ_NAME/src}"
app config -name $ECC_PROJ_NAME -add include-path "\${workspace_loc:/$FTL_PROJ_NAME/include}"

