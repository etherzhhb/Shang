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
			end
			end)
return s
end

BlockRAMInitFileGenScript=[=[
#local ram_num = 0
#for k,v in pairs(GlobalVariables) do
$(
	local mem_data = ''
  --Count in a row in order to form 64 bit for each row 
  local count = 0
if v.Initializer ~= nil then
		if v.ElemSize == 8 then
			for i,n in ipairs(v.Initializer) do
				count = count + 1
        --Line the varables from right to left
				mem_data = HexToBin(n,8)..mem_data
        --If the elemsize of v.Initializer is 8 then do like this---------
        --If each varable's width is 8 then 8 varables would form a row
				if count == 8 then
					_put(mem_data)
          --In order to debug we comment the first line with some markble words
					if i==8 then
            _put('   // -8- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
            --Insert each array's beginning address to a table
						table.insert(table_name, k)
						table.insert(table_num, ram_num)
					end
          --Give each line a '\n' except the last one
					if(i ~= v.NumElems) then
            _put('\n')
          else
            table.insert(LineTotal, ram_num)
					end
					count = 0
					mem_data = ''
          --Mark the number of each line then we can caluclate the address at the end of file
					ram_num = ram_num + 1
				end
        --If at the last varables the count is less than 8 we use '0' to make a line is 64 bits
				if (i == v.NumElems) then
					if count~=8 and count > 0 then
					  if v.NumElems > 8 then
							for j=1,((8-count)*8) do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -8- the end of ') _put(count) _put(k)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1
						elseif v.NumElems < 8 then
							for j=1,((8-count)*8) do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -8_- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
              --Insert each array's beginning address to a table
							table.insert(table_name, k)
							table.insert(table_num, ram_num)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1
						end
					end
				end
      end
    --If the elemsize of v.Initializer is 16 then do like this---------
		elseif v.ElemSize == 16 then
    --If each varable's width is 16 then 4 varables would form a row
			for i,n in ipairs(v.Initializer) do
				count = count + 1
        --Line the varables from right to left
				mem_data = HexToBin(n,16)..mem_data
				if count == 4 then
					_put(mem_data)
					if i == 4 then
            --In order to debug we comment the first line with some markble words
            _put('   // -16- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
            --Insert each array's beginning address to a table
						table.insert(table_name, k)
						table.insert(table_num, ram_num)
					end
          --Give each line a '\n' except the last one
					if(i ~= v.NumElems) then
            _put('\n')
          else
            table.insert(LineTotal, ram_num)
					end
					count = 0
					mem_data = ''
          --Mark the number of each line then we can caluclate the address at the end of file
					ram_num = ram_num + 1
				end

				if (i == v.NumElems) then
					if count~=4 and count > 0 then
					  if v.NumElems >4 then
							for j=1,((4-count)*16) do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -16- the end of ') _put(count) _put(k)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1
						else
							for j=1,((4-count)*16) do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -16_- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
              --Insert each array's beginning address to a table
							table.insert(table_name, k)
							table.insert(table_num, ram_num)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1		
						end
					end
				end
			end
    --if the elemsize of v.Initializer is 32 then do like this---------
		elseif v.ElemSize == 32 then
    --if each varable's width is 32 then 2 varables would form a row
			for i,n in ipairs(v.Initializer) do
				count = count + 1
        --Line the varables from right to left
				mem_data = HexToBin(n,32)..mem_data
				if count == 2 then
					_put(mem_data)
					if i==2 then
            --In order to debug we comment the first line with some markble words
            _put('   // -32- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
            --Insert each array's beginning address to a table
						table.insert(table_name, k)
						table.insert(table_num, ram_num)
					end
          --Give each line a '\n' except the last one
					if(i ~= v.NumElems) then
            _put('\n')
          else
            table.insert(LineTotal, ram_num)
					end
					count = 0
					mem_data = ''
          --Mark the number of each line then we can caluclate the address at the end of file
					ram_num = ram_num + 1
				end

				if (i == v.NumElems) then
					if count == 1 then
					  if v.NumElems >2 then
							for j=1,32 do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -32- the end of ') _put(count) _put(k)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1
						else
							for j=1,32 do	mem_data = '0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('    //-32_- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
              --Insert each array's beginning address to a table
							table.insert(table_name, k)
							table.insert(table_num, ram_num)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1		
						end
					end
				end
			end
    --if the elemsize of v.Initializer is 64 then do like this---------
		elseif v.ElemSize == 64 then
			for i,n in ipairs(v.Initializer) do
				_put(HexToBin(n,64))
				if i==1 then
          --In order to debug we comment the first line with some markble words
          _put('    //-64- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
          --Insert each array's beginning address to a table
					table.insert(table_name, k)
					table.insert(table_num, ram_num)
				end
        --Give each line a '\n' except the last one
				if(i ~= v.NumElems) then
          _put('\n')
        else
          table.insert(LineTotal, ram_num)
				end
        --Mark the number of each line then we can caluclate the address at the end of file
				ram_num = ram_num + 1
			end
		end
	else
    --if v.Initializer is nil then do like this-------------
    if v.ElemSize == 8 then
			for i = 1, v.NumElems do
				count = count + 1
        --Line the varables from right to left
				mem_data = HexToBin(0,8)..mem_data
				if count == 8 then
					_put(mem_data)
					if i==8 then
            _put('   // -8 nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
						table.insert(table_name, k)
						table.insert(table_num, ram_num)
					end
					if(i ~= v.NumElems) then
            _put('\n')
          else
            table.insert(LineTotal, ram_num)
					end
					count = 0
					mem_data = ''
					ram_num = ram_num + 1
				end
				if (i == v.NumElems) then
					if count~=8 and count > 0 then
					  if v.NumElems > 8 then
							for j=1,((8-count)*8) do	mem_data='0'..mem_data end
							_put(mem_data)
              _put('   // -8nil- the end of ') _put(count) _put(k)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
							ram_num = ram_num + 1
						elseif v.NumElems < 8 then
							for j=1,((8-count)*8) do	mem_data='0'..mem_data end
							_put(mem_data)
              _put('   // -8_nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
							table.insert(table_name, k)
							table.insert(table_num, ram_num)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
							ram_num = ram_num + 1
						end
					end
				end
      end
    elseif v.ElemSize == 16 then
    --If each varable's width is 16 then 4 varables would form a row
			for i = 1, v.NumElems do
				count = count + 1
        --Line the varables from right to left
				mem_data = HexToBin(0,16)..mem_data
				if count == 4 then
					_put(mem_data)
					if i == 4 then
            --In order to debug we comment the first line with some markble words
            _put('   // -16nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
            --Insert each array's beginning address to a table
						table.insert(table_name, k)
						table.insert(table_num, ram_num)
					end
          --Give each line a '\n' except the last one
					if(i ~= v.NumElems) then
            _put('\n')
          else
            table.insert(LineTotal, ram_num)
					end
					count = 0
					mem_data = ''
          --Mark the number of each line then we can caluclate the address at the end of file
					ram_num = ram_num + 1
				end

				if i == v.NumElems then
					if count~=4 and count > 0 then
					  if v.NumElems >4 then
							for j=1,((4-count)*16) do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -16nil- the end of ') _put(count) _put(k)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1
						else
							for j=1,((4-count)*16) do	mem_data='0'..mem_data end
							_put(mem_data)
              --In order to debug we comment the first line with some markble words
              _put('   // -16_nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
              --Insert each array's beginning address to a table
							table.insert(table_name, k)
							table.insert(table_num, ram_num)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
              --Mark the number of each line then we can caluclate the address at the end of file
							ram_num = ram_num + 1	
						end
					end
				end
			end
    elseif v.ElemSize == 32 then
			for i = 1, v.NumElems do
				count = count + 1
				mem_data = HexToBin(0,32)..mem_data
				if count == 2 then
					_put(mem_data)
					if i==2 then
            _put('   // -32nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
						table.insert(table_name, k)
						table.insert(table_num, ram_num)
					end
					if(i ~= v.NumElems) then
            _put('\n')
          else
            table.insert(LineTotal, ram_num)
					end
					count = 0
					mem_data = ''
					ram_num = ram_num + 1
				end

				if (i == v.NumElems) then
					if count == 1 then
					  if v.NumElems >2 then
							for j=1,32 do	mem_data='0'..mem_data end
							_put(mem_data)
              _put('   // -32nil- the end of ') _put(count) _put(k)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
							ram_num = ram_num + 1
						else
							for j=1,32 do	mem_data = '0'..mem_data end
							_put(mem_data)
              _put('    //-32_nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
							table.insert(table_name, k)
							table.insert(table_num, ram_num)
              table.insert(LineTotal, ram_num)
							count = 0
							mem_data = ''
							ram_num = ram_num + 1
						end
					end
				end
			end
    --if the elemsize of v.Initializer is 64 then do like this---------
		elseif v.ElemSize == 64 then
			for i = 1, v.NumElems do
				_put(HexToBin(0,64))
				if i==1 then
          _put('    //-64 nil- \`define gv') _put(k) _put(' 32\'d') _put(ram_num*8)
					table.insert(table_name, k)
					table.insert(table_num, ram_num)
				end
				if(i ~= v.NumElems) then
          _put('\n')
        else
          table.insert(LineTotal, ram_num)
				end
				ram_num = ram_num + 1
			end
		end
	end)
#end
]=]
