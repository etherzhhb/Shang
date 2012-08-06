function IntToBin(number,width)
	SaveBit={}
	number_reg = number
	local i=0
	if number == 0 then table.insert(SaveBit,1,0)
	elseif number>0 then
		while number~=0 do
			i=i+1
			if number%2==1  then table.insert(SaveBit,1,1) number=(number-1)/2
			elseif number%2==0 then table.insert(SaveBit,1,0) number=number/2
			end
		end
	else
		number=number*(-1)-1
		if number == 0 then table.insert(SaveBit,1,1)
		else
		while number~=0 do
			i=i+1
			if number%2==1  then table.insert(SaveBit,1,0) number=(number-1)/2
			elseif number%2==0 then table.insert(SaveBit,1,1) number=number/2
			end
			end
		end
	end
	local BitValue =''

	for index,value in ipairs(SaveBit) do  BitValue=BitValue..value end

	if number_reg>0 then
	 for j=1,(width-i) do
		BitValue='0'..BitValue
	 end
	elseif number_reg==0 then
	 for j=1,(width-1) do
		BitValue='0'..BitValue
	 end
	else
	 for j=1,(width-i) do
		BitValue='1'..BitValue
	 end
	end
	return BitValue
end

function HexToBin(hexstr,bitwidth)
	s=string.gsub(hexstr,"0x","")
	lenth = (string.len(s))
	Mustlenth = (bitwidth/4)
	Prex = ''
	if lenth < Mustlenth then
		for i=1,(Mustlenth-lenth) do
			Prex = Prex..'0'
		end
		s = Prex..s
	end
  s=string.gsub(s, "%x",
		function (c)
			if c=='0' then return '0000'
			elseif c=='1' then return '0001'
			elseif c=='2' then return '0010'
			elseif c=='3' then return '0011'
			elseif c=='4' then return '0100'
			elseif c=='5' then return '0101'
			elseif c=='6' then return '0110'
			elseif c=='7' then return '0111'
			elseif c=='8' then return '1000'
			elseif c=='9' then return '1001'
			elseif c=='A' then return '1010'
			elseif c=='B' then return '1011'
			elseif c=='C' then return '1100'
			elseif c=='D' then return '1101'
			elseif c=='E' then return '1110'
			elseif c=='F' then return '1111'
			elseif c=='a' then return '1010'
			elseif c=='b' then return '1011'
			elseif c=='c' then return '1100'
			elseif c=='d' then return '1101'
			elseif c=='e' then return '1110'
			elseif c=='f' then return '1111'
			end
			end)
return s
end

function linebyGVBit(k,v,table_name,table_num,LineTotal,strinit,addr)
  local Size = v.ElemSize
  local NumElems = v.NumElems
  local mem_data = ''
  --Count in a row in order to form 64 bit for each row 
  local count = 0
  local NumInLine = 64/Size
  for i=1, NumElems do
    count = count + 1
    --Line the varables from right to left
    if v.Initializer ~= nil then
      mem_data = HexToBin(v.Initializer[i],Size)..mem_data
    else
      mem_data = HexToBin(0,Size)..mem_data
    end
    --If the elemsize of v.Initializer is 8 then do like this---------
    --If each varable's width is 8 then 8 varables would form a row
    if count == NumInLine then
      if addr == 0 then 
        strinit = strinit..mem_data
      else
        strinit = strinit..'\n'..mem_data
      end
      --In order to debug we comment the first line with some markble words
      if i==NumInLine then
      --Insert each array's beginning addr to a table
        table.insert(table_name, k)
        table.insert(table_num, addr)
      end
      --Give each line a '\n' except the last one
      if(i == NumElems) then
        table.insert(LineTotal, addr)
      end
      count = 0
      mem_data = ''
      --Mark the number of each line then we can caluclate the addr at the end of file
      addr = addr + 1
      if(i == NumElems) then
        return strinit,addr
      end
    end
    --If at the last varables the count is less than 8 we use '0' to make a line is 64 bits
    if (i == NumElems) then
      if count~=NumInLine and count > 0 then
        if NumElems > NumInLine then
          for j=1,((NumInLine-count)*Size) do	mem_data='0'..mem_data end
          strinit = strinit..'\n'..mem_data
          table.insert(LineTotal, addr)
          count = 0
          mem_data = ''
          --Mark the number of each line then we can caluclate the addr at the end of file
          addr = addr + 1
          return strinit,addr
        elseif NumElems < NumInLine then
          for j=1,((NumInLine-count)*Size) do	mem_data='0'..mem_data end
          if addr == 0 then 
            strinit = strinit..mem_data
          else
            strinit = strinit..'\n'..mem_data
          end
          --Insert each array's beginning addr to a table
          table.insert(table_name, k)
          table.insert(table_num, addr)
          table.insert(LineTotal, addr)
          count = 0
          mem_data = ''
          --Mark the number of each line then we can caluclate the addr at the end of file
          addr = addr + 1
          return strinit,addr
        end
      end
    end
  end
end

function linebyGVBit64(k,v,table_name,table_num,LineTotal,strinit,addr)
  local WriteData = ''
  for i=1,v.NumElems do
    if v.Initializer ~= nil then
      WriteData = HexToBin(v.Initializer[i],64)
    else
      WriteData = HexToBin(0,64)
    end
    if addr == 0 then 
      strinit = strinit..WriteData
    else
      strinit = strinit..'\n'..WriteData
    end
    if i==1 then
      --Insert each array's beginning addr to a table
      table.insert(table_name, k)
      table.insert(table_num, addr)
    end
    --Give each line a '\n' except the last one
    if(i == v.NumElems) then
      table.insert(LineTotal, addr)
    end
    --Mark the number of each line then we can caluclate the addr at the end of file
    addr = addr + 1
    if(i == v.NumElems) then
      return strinit,addr
    end
  end
end

function placeGVinBramInitfile(k,v,strinit,addr,table_name,table_num,LineTotal)
  if v.ElemSize == 64 then
    return linebyGVBit64(k,v,table_name,table_num,LineTotal,strinit,addr)
  else
    return linebyGVBit(k,v,table_name,table_num,LineTotal,strinit,addr)
  end
end

BlockRAMInitFileGenScript=[=[
#local addr = 0
#local strinit = ''
#for k,v in pairs(GlobalVariables) do
#strinit,addr = placeGVinBramInitfile(k,v,strinit,addr,table_name,table_num,LineTotal)
#end
#_put(strinit)
#_put('\n')
]=]
