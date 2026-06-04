[Setup]
AppName=Configurador Aviot
AppVersion=1.0
AppPublisher=Ingeniatic Desarrollo S.L.
AppPublisherURL=https://aviot.es
DefaultDirName={autopf}\Configurador Aviot
DefaultGroupName=Aviot
OutputBaseFilename=Instalador_Configurador_Aviot
SetupIconFile=aviot.ico
WizardImageFile=wizard_image.bmp
WizardSmallImageFile=wizard_small.bmp
UninstallDisplayIcon={app}\Configurador_Aviot.exe
Compression=lzma
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=admin

[Languages]
Name: "spanish"; MessagesFile: "compiler:Languages\Spanish.isl"

[Files]
Source: "dist\Configurador_Aviot.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "drivers\CH341SER.EXE"; DestDir: "{app}\drivers"; Flags: ignoreversion

[Icons]
Name: "{group}\Configurador Aviot"; Filename: "{app}\Configurador_Aviot.exe"
Name: "{autodesktop}\Configurador Aviot"; Filename: "{app}\Configurador_Aviot.exe"

[Run]
Filename: "{app}\Configurador_Aviot.exe"; Flags: postinstall nowait skipifsilent
; Solo abre el driver si NO esta instalado
Filename: "{app}\drivers\CH341SER.EXE"; Flags: postinstall skipifsilent waituntilterminated; Check: not IsDriverInstalled

[Code]
function IsDriverInstalled: Boolean;
begin
  Result := RegKeyExists(HKLM, 'SYSTEM\CurrentControlSet\Services\CH341SER')
         or RegKeyExists(HKLM, 'SYSTEM\CurrentControlSet\Services\CH341SER_A64');
end;
