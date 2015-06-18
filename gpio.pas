unit gpio;

// RealityTech/DSI GPIO Lib for Pi
// See docs for useage
//
// We can now start a revision history as this is a useable library now!
// 18/06/2015 by RAI
// Bulkread bug issues, fix return data being gibberish due to buffer matching isues. ti2cstream needs conversion from dynamic array



{$mode objfpc}{$H+}

interface


uses
  Classes, SysUtils, baseunix, dialogs, math;


const
     GPIO_INIT_OK = 0;
     GPIO_INIT_INUSE = 1;
     // constants for HTU21D
     HTU_GETTEMP =    $f3;     // get temp with no hold
     HTU_GETHUM =     $f5;     // get humidity with no hold
     HTU_USR_WR =     $e6;     // write to reg
     HTU_USR_RD =     $e7;     // read from reg
     HTU_RESET =      $fe;     // reset
     HTU_SLEEPTIME =  50;      // we sleep for 50ms to allow for worst case scenario
                               // datasheet says that 20ms should be enough, it isnt!
     BMP_CMD_REG =    $f4;     // command register
     BMP_GET_TEMP =   $2E;     // bm085 start temp conversion
     BMP_GET_PRES =   $34;     // bmp085 start pressure read
     BMP_DATA     =   $f6;     // start of data reg
     BMP_CAL      =   $AA;     // start of cal register
     BMP_CAL_SIZE =   22;      // size of cal reg

type
  ti2cstream = array of byte;
  tirqready = procedure(irqnr:integer;dir:boolean) of object;
  tbmp085_rec = record                              // here be dragons!
                ac1, ac2, ac3, b1, b2, mb, mc, md : word;
                AC4,AC5,AC6,UT : word;
                th,tl,ph,pl,pxl : byte;
                rawtemp, rawpres : integer;
                pres,temp : real;
                valid : boolean;
                end;

  thtu_record = record
              templ,temph,huml,humh : byte;         // raw data from the sensor
              rawh,rawt : word;                     // expanded 16 bit vals
              modeword : byte;                      // modeword, we need this to know the resolution
              cks : byte;                           // checksum., for now, binned
              rh,temp,c_rh : real;                  // real output values
              valid : boolean;                      // is the dataset valid
    end;

  tirqthread = class (tthread)
    private
      fonirqready : TIRQready;
      hold : integer;
      procedure died;
    protected
      procedure execute; override;
    public
      constructor Create (createsuspended : boolean);
      property OnIRQ : TIRQready read fonirqready write fonirqready;
   end;

    gpio_rec = record
       name : string;
       pin : integer;
       enabled : boolean;
       dir : boolean;
       irq : boolean;
       oldstate : boolean;
       exported : boolean;
       locked : boolean;
       pullup : boolean;
       value : boolean;
       end;
  novar = char;

  function getfsstate (gpio:integer):gpio_rec;
  function gpioinit (force:boolean): integer;
  function gpiounexport(pin:integer):boolean;
  function gpiosetdir(pin:integer;inout:boolean):boolean;
  function gpioexport(pin:integer):boolean;
  function gpioread (gpiopin:integer): boolean;
  function getgpiobyname(name:string):integer;
  function gpioset(gpiopin:integer;value:boolean):boolean;
  function SPIenable:boolean;
  function UARTenable:boolean;
  function I2Cenable(device:string; target:cint; var fhandle:integer):boolean;
  function I2Cdisable(var fhandle:integer):boolean;
  function lockgpio(pin:integer):boolean;
  function SPIdisable:boolean;
  function UARTdisable:boolean;
  function unlockgpio(pin:integer):boolean;
  procedure startIRQ(irqproc:tirqready; holdoff:integer);
  function writei2c(reg,data:byte; var fhandle:integer) : boolean;
  function writei2craw(data:byte; var fhandle:integer) : boolean;
  function readi2c (reg,count : byte; fhandle:integer):ti2cstream;
  function readi2creg (reg: byte; fhandle:integer):integer;
  function readi2craw (fhandle:integer):integer;
  function htu21init (device:string; target:cint; var fhandle:integer):boolean;
  function htu21read (var fhandle:integer):thtu_record;
  function bmp085init (device:string; target:cint; var fhandle:integer):boolean;
  function bmp085read (var fhandle:integer): tbmp085_rec;



var
   gpios :array[2..27] of gpio_rec;   // all our GPIO pins. Start at 2 as 1 is used somewhere else
   irqthread : tirqthread;
   i2cbuff : array [1..255] of byte;  // use by I2c routines
   caldata : tbmp085_rec;
implementation


// IRQ Generation thread
procedure startIRQ(irqproc:tirqready; holdoff :integer);
var
   loop : integer;
begin
     //if assigned(irqthread) then exit;
     // read all pins NOW!
     for loop := 2 to 27 do begin
              if (gpios[loop].enabled) and (gpios[loop].dir) and (gpios[loop].irq) and not (gpios[loop].locked) then gpios[loop].oldstate:= gpioread(loop);
     end;
     irqthread := tirqthread.create(true);
     irqthread.hold:=holdoff;
     irqthread.OnIRQ:=irqproc;
     irqthread.start;
end;

constructor tirqthread.create (createsuspended : boolean);
begin
            freeonterminate := true;
            inherited create (createsuspended);
end;

procedure tirqthread.died;
begin
            messagedlg ('Thread died!',mtinformation,[mbok],0);
end;

procedure tirqthread.execute;
var
   loop : integer;
   pinstate : boolean;
begin
     repeat
     for loop := 2 to 27 do begin
         if (gpios[loop].enabled) and (gpios[loop].dir) and (gpios[loop].irq) and not (gpios[loop].locked) then begin
           pinstate := gpioread(loop);
           if pinstate <> gpios[loop].oldstate then begin
             if gpios[loop].oldstate > pinstate then onirq(loop,false) else onirq(loop,true);
             gpios[loop].oldstate := pinstate;
           end;
       end;
     end;
     sleep(hold);
     until terminated ;
     died;
end;

// enable SPI
function SPIenable:boolean;
begin
    // we dont support SPI just yet but if its in use we need to enable it in here to map out the IO pins
    // note, this will only map out STANDARD pins!
    lockgpio(7);
    lockgpio(8);
    lockgpio(9);
    lockgpio(10);
    lockgpio(11);
    result := true;
end;

function SPIdisable:boolean;
begin
    // we dont support SPI just yet but if its in use we need to enable it in here to map out the IO pins
    // note, this will only map out STANDARD pins!
    unlockgpio(7);
    unlockgpio(8);
    unlockgpio(9);
    unlockgpio(10);
    unlockgpio(11);
    result := true;
end;


function UARTenable:boolean;
begin
    // we dont support UART as there are easier ways but we need to enable it in here to map out the IO pins
    // note, this will only map out STANDARD pins!
    lockgpio(14);
    lockgpio(15);
    result := true;
end;

function UARTdisable:boolean;
begin
    unlockgpio(14);
    unlockgpio(15);
    result := true;
end;

// raw bus read
function readi2craw (fhandle:integer):integer;
var
    data : array [1..255] of byte;
    readcount  : integer;
begin
result := -1;
     if fhandle = -1 then begin
                   exit;
                   end;
     readcount := fpread(fhandle,data,1);
     if readcount <> 1 then begin
              result := -1;
              exit;
              end;
     result := data[1];
end;

// read a register. We cant do repeated start so bulkread may be your best bet
function readi2creg (reg: byte; fhandle:integer):integer;
var
    data : array [1..255] of byte;
    readcount  : integer;
begin
result := -1;
     if fhandle = -1 then begin
                   exit;
                   end;
     readcount := fpwrite(fhandle,reg,1);
     if readcount <> 1 then begin
       messagedlg ('Write failed',mtinformation,[mbok],0);

              exit;
     end;
     readcount := fpread(fhandle,data,1);
     if readcount <> 1 then begin
              result := -1;
              exit;
              end;
     result := data[1];
end;

//enable I2C
function I2Cenable(device:string; target:cint; var fhandle:integer):boolean;
begin
    lockgpio(2);
    lockgpio(3);
  result := true;
  fhandle:= fpopen(device,O_RDWR);
  fpioCTL(fhandle,1795, pointer(target));
  if fhandle= -1 then begin
              result := false;
              exit;
              end;
end;

// BMP085 init. This is a bit more involved as we need to grab all the cal data
// This sensor is an utter pain in the ass but it is VERY accurate
function bmp085init (device:string; target:cint; var fhandle:integer):boolean;
var
         data : ti2cstream;

begin
  // for the best part we are just going to do an init to start
  // no point duplicating things
  if not i2cenable(device,target,fhandle) then begin
     result := false;     // if we cont open the bus then no point going on
     exit;
     end;
  // now wake up the sensor
  if not writei2craw(HTU_RESET,fhandle) then begin
     result := false;     // if the init failed then bomb out
     exit;
  end;
 // if we get here, in theory the device is up and ready to go
 // bmp has a massive (and complex) cal routine. We now need to go and grab a huge chunk of ROM
 data :=  readi2c(BMP_CAL,BMP_CAL_SIZE,fhandle);
 if length(data) = 0 then begin      // no cal data!
                result := false;
                end;
 messagedlg('I got '+inttostr(length(data))+' bytes',mtinformation,[mbok],0);
 // and now pick out the bits. data[0] = 0xaa      hi
 wordrec(caldata.ac1).hi := data[0];
 wordrec(caldata.ac1).lo := data[1];
 wordrec(caldata.ac2).hi := data[2];
 wordrec(caldata.ac2).lo := data[3];
 wordrec(caldata.ac3).hi := data[4];
 wordrec(caldata.ac3).lo := data[5];
 wordrec(caldata.ac4).hi := data[6];
 wordrec(caldata.ac4).lo := data[7];
 wordrec(caldata.ac5).hi := data[8];
 wordrec(caldata.ac5).lo := data[9];
 wordrec(caldata.ac6).hi := data[10];
 wordrec(caldata.ac6).lo := data[11];
 wordrec(caldata.b1).hi := data[12];
 wordrec(caldata.b1).lo := data[13];
 wordrec(caldata.b2).hi := data[14];
 wordrec(caldata.b2).lo := data[15];
 wordrec(caldata.mb).hi := data[16];
 wordrec(caldata.mb).lo := data[17];
 wordrec(caldata.mc).hi := data[18];
 wordrec(caldata.mc).lo := data[19];
 wordrec(caldata.md).hi := data[20];
 wordrec(caldata.md).lo := data[21];
 if (caldata.ac1 = 0) or (caldata.ac1 =$ff) then messagedlg ('AC1 is invalid',mterror,[mbok],0) ;
  if (caldata.ac2 = 0) or (caldata.ac2 =$ff) then messagedlg ('AC2 is invalid',mterror,[mbok],0) ;
   if (caldata.ac3 = 0) or (caldata.ac3 =$ff) then messagedlg ('AC3 is invalid',mterror,[mbok],0) ;
    if (caldata.ac4 = 0) or (caldata.ac4 =$ff) then messagedlg ('AC4 is invalid',mterror,[mbok],0) ;
     if (caldata.ac5 = 0) or (caldata.ac5 =$ff) then messagedlg ('AC5 is invalid',mterror,[mbok],0) ;
      if (caldata.ac6 = 0) or (caldata.ac6 =$ff) then messagedlg ('AC6 is invalid',mterror,[mbok],0) ;
       if (caldata.b1 = 0) or (caldata.b1 =$ff) then messagedlg ('B1 is invalid',mterror,[mbok],0) ;
        if (caldata.b2 = 0) or (caldata.b2 =$ff) then messagedlg ('B2 is invalid',mterror,[mbok],0) ;
         if (caldata.mb = 0) or (caldata.mb =$ff) then messagedlg ('MB is invalid',mterror,[mbok],0) ;
          if (caldata.mc = 0) or (caldata.mc =$ff) then messagedlg ('MC is invalid',mterror,[mbok],0) ;
           if (caldata.md = 0) or (caldata.md =$ff) then messagedlg ('MD is invalid',mterror,[mbok],0) ;
 // we should validate in here now. This can go in later but long and short is nothing should be 0 or ff
 //all done for now!
 result := true;
 end;

// grab temp and pressure data and do unholy math with them :(
function bmp085read (var fhandle:integer): tbmp085_rec;
var
   data : ti2cstream;
   x1,x2,b5 : longint;
   cac5 : longint;
begin
     // first up check the handle
  if fhandle = -1 then begin
   result.valid := false;
   exit;
   end;
  result := caldata; // grab all our cal data
  if not writei2c(BMP_CMD_REG,BMP_GET_TEMP,fhandle) then begin
     result.valid := false;
     messagedlg('Kaboom! : Init temp conv failed ',mterror,[mbok],0);
     exit;
  end;
  // we should wait here for a sec. Datasheet says4.5ms. I'm going for 10
  sleep (10);
  data :=  readi2c(BMP_DATA,2,fhandle); // get temp words
  if length(data) =0 then begin
     result.valid := false;
     messagedlg('Kaboom! : temp read failed ',mterror,[mbok],0);
     exit;
  end;
  result.rawtemp:=data[0] SHL 8 + data[1];
  // We now *should* have the temp, get pressure
  if not writei2c(BMP_CMD_REG,BMP_GET_PRES,fhandle) then begin
     result.valid := false;
     messagedlg('Kaboom! : Init pressure conv failed ',mterror,[mbok],0);
     exit;
  end;
  // we should wait here for a sec. Datasheet says4.5ms. I'm going for 10
  sleep (10);
  data :=  readi2c(BMP_DATA,3,fhandle); // get pressure words, 3 this time
  if length(data) = 0 then begin
     result.valid := false;
     messagedlg('Kaboom! : pres read failed ',mterror,[mbok],0);
     exit;
  end;
  result.rawpres:=data[0] SHL 16 + data[1] shl 8 + data[2];
  // now in theory we have all we need...here we go, right from the datasheet...
  // temp ....
  cac5 := result.ac5;
  x1 := (result.rawtemp-result.AC6)*cac5 div 65536; //power(2,16);
  messagedlg ('X1= '+inttostr(x1),mtinformation,[mbok],0);
  x2 := result.mc * 2048 div(x1+result.md);
  messagedlg ('X2= '+inttostr(x2),mtinformation,[mbok],0);
  b5 := x1+x2;
  result.temp:= (b5+8)/power(2,4);
  messagedlg ('Temp : '+floattostrf(result.temp/10,fffixed,2,2),mtinformation,[mbok],0);

end;

// open the bus, init the HTU21D and set it off ready to go.
// Multiple devices are allowed
function htu21init (device:string; target:cint; var fhandle:integer):boolean;
var
         procresult : boolean;
begin
  // for the best part we are just going to do an init to start
  // no point duplicating things
  if not i2cenable(device,target,fhandle) then begin
     result := false;     // if we cont open the bus then no point going on
     exit;
     end;
  // now wake up the sensor
  if not writei2craw(HTU_RESET,fhandle) then begin
     result := false;     // if the init failed then bomb out
     exit;
  end;
 // if we get here, in theory the device is up and ready to go
 result := true;
end;

function htu21read (var fhandle:integer):thtu_record;
var
   htubuff:array [0..4] of byte;
   readcount : integer;
   temp : word;
begin
// first up check the handle
  if fhandle = -1 then begin
   result.valid := false;
   exit;
   end;
 // start a temp measurement...
 if not writei2craw(HTU_GETTEMP,fhandle) then begin
   result.valid := false;
   exit;
 end;
// wait for measurement, we could poll this and probobly will later on
sleep(HTU_SLEEPTIME);
// we now need three bytes. As we want three bytes in bulk we will do the read here
readcount := fpread(fhandle,htubuff,3);
if readcount <> 3 then begin
         result.valid := false;
         exit;
         end;
result.temph:=htubuff[0];
result.templ:=htubuff[1];
// at this time we bin the checksum
// start a humidity measurement...
if not writei2craw(HTU_GETHUM,fhandle) then begin
  result.valid := false;
  exit;
end;
// wait for measurement, we could poll this and probobly will later on
sleep(HTU_SLEEPTIME);
// we now need three bytes. As we want three bytes in bulk we will do the read here
readcount := fpread(fhandle,htubuff,3);
if readcount <> 3 then begin
        result.valid := false;
        exit;
        end;
result.humh:=htubuff[0];
result.huml:=htubuff[1];
// at this time we bin the checksum
result.valid := true;
// right math time!  We want 16 bit data to work with...
wordrec(result.rawh).Lo := result.huml;
wordrec(result.rawh).hi := result.humh;
wordrec(result.rawt).Lo := result.templ;
wordrec(result.rawt).hi := result.temph;
// temp
result.temp:=-46.85+175.72*result.rawt/power(2,16);
// humidity
result.rh:= -6+125*result.rawh/power(2,16);
// Temp Compensation
result.c_rh:=result.rh+(25-result.temp)*-0.15;


end;

function I2Cdisable(var fhandle:integer):boolean;
begin
      if fhandle = -1 then begin
       result := false;
       exit;
       end;
     try
        fpclose(fhandle);
     except
           result := false;
           exit;
     end;
     result := true;
//     unlockgpio(2);   we used to unlock here but as we can have multiple
//     unlockgpio(3);   devices open, that would be silly!
     result := true;
     fhandle := -1;
end;

// raw write (command mode)
function writei2craw(data:byte; var fhandle:integer) : boolean;
var
    buffer : array [0..10] of byte;
    count : integer;
begin
     if fhandle = -1 then begin
       result := false;
       messagedlg ('Write failed, device is not assigned',mtinformation,[mbok],0);
       exit;
       end;
    // try
     result := true;
     buffer[0] := data;
     count := fpwrite(fhandle,buffer[0],1);
     if count <> 1 then begin
              result := false;
              messagedlg ('Write failed. Code:'+inttostr(fpgeterrno),mtinformation,[mbok],0);
              exit;
              end
     else result := true;
 end;

// write a register
function writei2c(reg,data:byte; var fhandle:integer) : boolean;
var
    buffer : array [0..10] of byte;
    count : integer;
begin


     if fhandle = -1 then begin
       result := false;
       messagedlg ('Write failed, device is not assigned',mtinformation,[mbok],0);
       exit;
       end;
    // try
     result := true;
     buffer[0] := reg;
     buffer[1] := data;
     count := fpwrite(fhandle,buffer[0],2);
     if count <> 2 then begin
              result := false;
              messagedlg ('Write failed. Code:'+inttostr(fpgeterrno),mtinformation,[mbok],0);
              exit;
              end
     else result := true;
 end;



function readi2c (reg,count : byte; fhandle:integer):ti2cstream;
var
    data : array [1..255] of byte;
    readcount  : integer;
begin
     setlength(result,0);
     if fhandle = -1 then begin
                   exit;
                   end;
     readcount := fpwrite(fhandle,reg,1);
     if readcount <> 1 then begin
              exit;
     end;
     readcount := fpread(fhandle,data,count);
     if readcount <> count then begin
              setlength(result,0);
              exit;
              end;
     setlength(result,1);
     result[0] := data[1];
end;

//lock out a pin
function lockgpio(pin:integer):boolean;
begin
     gpios[pin].name := 'LOCKED';
     gpios[pin].enabled:= false;
     gpios[pin].locked:=true;
end;

//unlock a pin
function unlockgpio(pin:integer):boolean;
begin
     gpios[pin].name := 'GPIO'+inttostr(pin);
     gpios[pin].enabled:= false;
     gpios[pin].locked:=false;
end;


// get GPIO state from filesystem
function getfsstate (gpio:integer):gpio_rec;
var
filedesc : integer;
buttonstatus : string[1] ='1';
returncode : integer;
begin
     if not fileexists('/sys/class/gpio/gpio'+inttostr(gpio)) then begin   // not exported so no info
       result.exported:=false;
       exit;
     end
     else begin
       result.exported := true;
       result.name := 'gpio'+inttostr(gpio);
       result.value:=gpioread(gpio);
       try  // value
            filedesc := fpopen('/sys/class/gpio/gpio'+inttostr(gpio)+'/direction', o_rdonly);
            if filedesc > 0 then
               begin
                 returncode := fpread (filedesc, buttonstatus[1],1);
                 if upcase(buttonstatus) ='I' then result.dir := true else result.dir := false;
                 end;
            finally
            returncode := fpclose(filedesc);
       end;
 end;

end;

// get a GPIO pin by name (case insensitive)
function getgpiobyname(name:string):integer;
var
         loop : integer;
begin
     result := -1;
     if uppercase(name) = 'LOCKED' then exit;
     for loop := 2 to 27 do
         if uppercase(gpios[loop].name) = uppercase(name) then begin
            result := loop;
            exit;
            end;
end;


// go through and grab the state of all pins, set array up etc
function gpioinit (force:boolean): integer;
var
   filehandle, returncode : longint;
   loop : integer;
   temp_pin : gpio_rec;
begin
     for loop := 2 to 27 do begin
         gpios[loop].name:='GPIO'+inttostr(loop);
         if fileexists('/sys/class/gpio/gpio'+inttostr(loop)) then begin
            if force then begin
               gpios[loop].exported:=true;
               gpios[loop].locked:= false;
               temp_pin := getfsstate(loop);
               gpios[loop].dir:=temp_pin.dir;
               gpios[loop].enabled := true;
               gpios[loop].value:=temp_pin.value;
               result := GPIO_INIT_INUSE;
            end
            else begin
               gpios[loop].exported:=true;
               gpios[loop].locked:=true;
               gpios[loop].name:='LOCKED';
               result := GPIO_INIT_INUSE;
            end;
         end;
     end;
     // start IRQ threads...
end;

function gpiounexport(pin:integer):boolean;
var
  filehandle, returncode : longint;
begin
     try
        // range check
        if pin < 1 then begin
           messagedlg ('Out of range!',mterror,[mbok],0);
           result := false;
           exit;
           end;
        if pin >27 then begin
           messagedlg ('Out of range!',mterror,[mbok],0);
           result := false;
           exit;
           end;
        if gpios[pin].locked then begin
           messagedlg ('Pin Locked',mterror,[mbok],0);
            result := false;
            exit;
            end;
        if not gpios[pin].enabled then begin
           messagedlg ('not Enabled',mterror,[mbok],0);
           result := false;
           exit;
           end;

      except
            result := false;
            exit;
      end;
      try
      filehandle := fpopen ('/sys/class/gpio/unexport',O_WRonly);
      if filehandle <= 0 then begin
                    result := false;
                    messagedlg ('Cant open file',mterror,[mbok],0);
                    exit;
                    end;
      returncode := fpwrite (filehandle,pchar(inttostr(pin)),length(inttostr(pin)));
      {if returncode <> 0 then begin
                    fpclose(filehandle);
                    messagedlg ('Unexport failed '+inttostr(returncode),mterror,[mbok],0);
                    result := false;
                    exit;
                    end;  }
      finally
      fpclose(filehandle);
      result := true;
      gpios[pin].enabled:=false;
      end;
end;


function gpioset(gpiopin:integer;value:boolean):boolean;
var
  filedesc : integer;
  dir : string;
  returncode : integer;
begin
        // range check
        if gpiopin < 2 then begin
           result := false;
           exit;
           end;
        if gpiopin >27 then begin
           result := false;
           exit;
           end;
        if gpios[gpiopin].locked then begin
            result := false;
            exit;
            end;
        if not gpios[gpiopin].enabled then begin
           result := false;
           exit;
           end;
     if value then dir := '1' else dir := '0';
      try
     filedesc := fpopen('/sys/class/gpio/gpio'+inttostr(gpiopin)+'/value', o_wronly);
     returncode := fpwrite (filedesc, pchar(dir),length(dir));
     finally
     returncode := fpclose(filedesc);
     end;
end;

function gpiosetdir(pin:integer;inout:boolean):boolean;
var
  filedesc : integer;
  dir : string;
  returncode : integer;
begin
        // range check
        if pin < 1 then begin
           result := false;
           exit;
           end;
        if pin >27 then begin
           result := false;
           exit;
           end;
        if gpios[pin].locked then begin
            result := false;
            exit;
            end;
        if not gpios[pin].enabled then begin
           result := false;
           exit;
end;
     if inout then dir := 'in' else dir := 'out';
      try
     filedesc := fpopen('/sys/class/gpio/gpio'+inttostr(pin)+'/direction', o_wronly);
     returncode := fpwrite (filedesc, pchar(dir),length(dir));
     finally
     returncode := fpclose(filedesc);
     end;
     gpios[pin].dir:=inout;
end;

function gpioexport(pin:integer):boolean;
var
  filehandle, returncode : longint;
begin
        // range check
        if pin < 1 then begin
           result := false;
           exit;
           end;
        if pin >27 then begin
           result := false;
           exit;
           end;
        if gpios[pin].locked then begin
           result := false;
           exit;
           end;
        if gpios[pin].enabled then begin
           result := false;
           exit;
           end;

try
      filehandle := fpopen ('/sys/class/gpio/export',O_WRonly);
      if filehandle <= 0 then begin
                    result := false;
                    exit;
                    end;
      returncode := fpwrite (filehandle,pchar(inttostr(pin)),length(inttostr(pin)));
      if returncode <> 0 then begin
                    fpclose(filehandle);
                    result := false;
                    exit;
                    end;
      finally
      fpclose(filehandle);
      gpios[pin].enabled := true;
      if gpiosetdir(pin,true) then gpios[pin].dir:=true;
      result := true;
      end;
end;

function gpioread (gpiopin:integer): boolean;
var
   filedesc : integer;
   buttonstatus : string[1] ='1';
   returncode : integer;
begin
         if gpiopin < 1 then begin
            result := false;
            exit;
            end;
         if gpiopin >27 then begin
            result := false;
            exit;
            end;
         if gpios[gpiopin].locked then begin
            result := false;
            exit;
            end;
        if not gpios[gpiopin].enabled then begin
           result := false;
           exit;
           end;

       try
            filedesc := fpopen('/sys/class/gpio/gpio'+inttostr(gpiopin)+'/value', o_rdonly);
            if filedesc > 0 then
               begin
                 returncode := fpread (filedesc, buttonstatus[1],1);
                 if buttonstatus ='1' then result := true else result := false;
                 gpios[gpiopin].value:=result;
                 end;
            finally
            returncode := fpclose(filedesc);
       end;

end;

end.

