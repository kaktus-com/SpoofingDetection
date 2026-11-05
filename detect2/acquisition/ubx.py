"""Offline UBX framing and RAWX observations; unknown messages stay archived."""
import struct

NAMES={(2,21):'RXM-RAWX',(2,19):'RXM-SFRBX',(1,53):'NAV-SAT',(1,7):'NAV-PVT',
       (13,1):'TIM-TP',(10,54):'MON-COMMS',(10,4):'MON-VER'}


def packet(cls, ident, payload):
    body=bytes([cls,ident])+struct.pack('<H',len(payload))+payload
    a=b=0
    for value in body:a=(a+value)&255;b=(b+a)&255
    return b'\xb5\x62'+body+bytes([a,b])


def frames(data, diagnostics):
    position=0;diagnostics.update(valid_frames=0,invalid_candidates=0,trailing_partial_candidates=0)
    while True:
        i=data.find(b'\xb5\x62',position)
        if i<0:break
        if i+6>len(data):diagnostics['trailing_partial_candidates']+=1;break
        length=int.from_bytes(data[i+4:i+6],'little');end=i+8+length
        if end>len(data):
            diagnostics['trailing_partial_candidates']+=1;position=i+1;continue
        a=b=0
        for value in data[i+2:end-2]:a=(a+value)&255;b=(b+a)&255
        if data[end-2:end]!=bytes([a,b]):
            diagnostics['invalid_candidates']+=1;position=i+1;continue
        diagnostics['valid_frames']+=1
        yield i,data[i+2],data[i+3],bytes(data[i+6:end-2])
        position=end


def rawx(payload):
    if len(payload)<16:raise ValueError('Short RAWX header')
    tow,week,leaps,count,status,version=struct.unpack_from('<dHbBBB',payload)
    if len(payload)!=16+32*count:raise ValueError('RAWX observation count/length mismatch')
    header={'gps_tow_s':tow,'gps_week':week,'leap_seconds':leaps,'receiver_status':status,
            'clock_reset':bool(status&2),'leap_seconds_valid':bool(status&1),'version':version,'num_meas':count}
    observations=[]
    for i in range(count):
        values=struct.unpack_from('<ddfBBBBHBBBBBB',payload,16+32*i)
        pr,cp,doppler,gnss,sv,sig,freq,lock,cno,prstd,cpstd,dostd,tracking,reserved=values
        observations.append({**header,'signal_id':f'{gnss}:{sv}:{sig}', 'gnss_id':gnss,'sv_id':sv,
            'signal_code':sig,'frequency_id':freq,'pseudorange_m':pr,'carrier_phase_cycles':cp,
            'doppler_hz':doppler,'lock_time_ms':lock,'cno_db_hz':cno,
            'pseudorange_std_encoding':prstd,'carrier_std_encoding':cpstd,'doppler_std_encoding':dostd,
            'tracking_status':tracking,'pseudorange_valid':bool(tracking&1),'carrier_valid':bool(tracking&2),
            'half_cycle_valid':bool(tracking&4),'sub_half_cycle':bool(tracking&8)})
    return header,observations
