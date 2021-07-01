/*
* MIT License
* 
* Copyright (c) 2021, Juha Heiskala
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
**/

		  
#include <stddef.h>
#include "libqhull_r.h"

extern size_t jl_qhull_qhT_size(void) {
    return sizeof(qhT);
}

extern size_t jl_qhull_facetT_size(void) {
    return sizeof(facetT);
}

extern size_t jl_qhull_vertexT_size(void) {
    return sizeof(vertexT);
}

extern double jl_qhull_rbox_out_offset(qhT* qh) {
    return qh->rbox_out_offset;
}

extern void* jl_qhull_facet_vertices(facetT* facet) {
    return facet->vertices;
}

extern void jl_qhull_qhT_offsets(size_t *vv) {
vv[0] = offsetof(qhT, ALLpoints);
vv[1] = offsetof(qhT, ALLOWshort);
vv[2] = offsetof(qhT, ALLOWwarning);
vv[3] = offsetof(qhT, ALLOWwide);
vv[4] = offsetof(qhT, ANGLEmerge);
vv[5] = offsetof(qhT, APPROXhull);
vv[6] = offsetof(qhT, MINoutside);
vv[7] = offsetof(qhT, ANNOTATEoutput);
vv[8] = offsetof(qhT, ATinfinity);
vv[9] = offsetof(qhT, AVOIDold);
vv[10] = offsetof(qhT, BESToutside);
vv[11] = offsetof(qhT, CDDinput);
vv[12] = offsetof(qhT, CDDoutput);
vv[13] = offsetof(qhT, CHECKduplicates);
vv[14] = offsetof(qhT, CHECKfrequently);
vv[15] = offsetof(qhT, premerge_cos);
vv[16] = offsetof(qhT, postmerge_cos);
vv[17] = offsetof(qhT, DELAUNAY);
vv[18] = offsetof(qhT, DOintersections);
vv[19] = offsetof(qhT, DROPdim);
vv[20] = offsetof(qhT, FLUSHprint);
vv[21] = offsetof(qhT, FORCEoutput);
vv[22] = offsetof(qhT, GOODpoint);
vv[23] = offsetof(qhT, GOODpointp);
vv[24] = offsetof(qhT, GOODthreshold);
vv[25] = offsetof(qhT, GOODvertex);
vv[26] = offsetof(qhT, GOODvertexp);
vv[27] = offsetof(qhT, HALFspace);
vv[28] = offsetof(qhT, ISqhullQh);
vv[29] = offsetof(qhT, IStracing);
vv[30] = offsetof(qhT, KEEParea);
vv[31] = offsetof(qhT, KEEPcoplanar);
vv[32] = offsetof(qhT, KEEPinside);
vv[33] = offsetof(qhT, KEEPmerge);
vv[34] = offsetof(qhT, KEEPminArea);
vv[35] = offsetof(qhT, MAXcoplanar);
vv[36] = offsetof(qhT, MAXwide);
vv[37] = offsetof(qhT, MERGEexact);
vv[38] = offsetof(qhT, MERGEindependent);
vv[39] = offsetof(qhT, MERGING);
vv[40] = offsetof(qhT, premerge_centrum);
vv[41] = offsetof(qhT, postmerge_centrum);
vv[42] = offsetof(qhT, MERGEpinched);
vv[43] = offsetof(qhT, MERGEvertices);
vv[44] = offsetof(qhT, MINvisible);
vv[45] = offsetof(qhT, NOnarrow);
vv[46] = offsetof(qhT, NOnearinside);
vv[47] = offsetof(qhT, NOpremerge);
vv[48] = offsetof(qhT, ONLYgood);
vv[49] = offsetof(qhT, ONLYmax);
vv[50] = offsetof(qhT, PICKfurthest);
vv[51] = offsetof(qhT, POSTmerge);
vv[52] = offsetof(qhT, PREmerge);
vv[53] = offsetof(qhT, PRINTcentrums);
vv[54] = offsetof(qhT, PRINTcoplanar);
vv[55] = offsetof(qhT, PRINTdim);
vv[56] = offsetof(qhT, PRINTdots);
vv[57] = offsetof(qhT, PRINTgood);
vv[58] = offsetof(qhT, PRINTinner);
vv[59] = offsetof(qhT, PRINTneighbors);
vv[60] = offsetof(qhT, PRINTnoplanes);
vv[61] = offsetof(qhT, PRINToptions1st);
vv[62] = offsetof(qhT, PRINTouter);
vv[63] = offsetof(qhT, PRINTprecision);
vv[64] = offsetof(qhT, PRINTout);
vv[65] = offsetof(qhT, PRINTridges);
vv[66] = offsetof(qhT, PRINTspheres);
vv[67] = offsetof(qhT, PRINTstatistics);
vv[68] = offsetof(qhT, PRINTsummary);
vv[69] = offsetof(qhT, PRINTtransparent);
vv[70] = offsetof(qhT, PROJECTdelaunay);
vv[71] = offsetof(qhT, PROJECTinput);
vv[72] = offsetof(qhT, RANDOMdist);
vv[73] = offsetof(qhT, RANDOMfactor);
vv[74] = offsetof(qhT, RANDOMa);
vv[75] = offsetof(qhT, RANDOMb);
vv[76] = offsetof(qhT, RANDOMoutside);
vv[77] = offsetof(qhT, REPORTfreq);
vv[78] = offsetof(qhT, REPORTfreq2);
vv[79] = offsetof(qhT, RERUN);
vv[80] = offsetof(qhT, ROTATErandom);
vv[81] = offsetof(qhT, SCALEinput);
vv[82] = offsetof(qhT, SCALElast);
vv[83] = offsetof(qhT, SETroundoff);
vv[84] = offsetof(qhT, SKIPcheckmax);
vv[85] = offsetof(qhT, SKIPconvex);
vv[86] = offsetof(qhT, SPLITthresholds);
vv[87] = offsetof(qhT, STOPadd);
vv[88] = offsetof(qhT, STOPcone);
vv[89] = offsetof(qhT, STOPpoint);
vv[90] = offsetof(qhT, TESTpoints);
vv[91] = offsetof(qhT, TESTvneighbors);
vv[92] = offsetof(qhT, TRACElevel);
vv[93] = offsetof(qhT, TRACElastrun);
vv[94] = offsetof(qhT, TRACEpoint);
vv[95] = offsetof(qhT, TRACEdist);
vv[96] = offsetof(qhT, TRACEmerge);
vv[97] = offsetof(qhT, TRIangulate);
vv[98] = offsetof(qhT, TRInormals);
vv[99] = offsetof(qhT, UPPERdelaunay);
vv[100] = offsetof(qhT, USEstdout);
vv[101] = offsetof(qhT, VERIFYoutput);
vv[102] = offsetof(qhT, VIRTUALmemory);
vv[103] = offsetof(qhT, VORONOI);
vv[104] = offsetof(qhT, AREAfactor);
vv[105] = offsetof(qhT, DOcheckmax);
vv[106] = offsetof(qhT, feasible_string);
vv[107] = offsetof(qhT, feasible_point);
vv[108] = offsetof(qhT, GETarea);
vv[109] = offsetof(qhT, KEEPnearinside);
vv[110] = offsetof(qhT, hull_dim);
vv[111] = offsetof(qhT, input_dim);
vv[112] = offsetof(qhT, num_points);
vv[113] = offsetof(qhT, first_point);
vv[114] = offsetof(qhT, POINTSmalloc);
vv[115] = offsetof(qhT, input_points);
vv[116] = offsetof(qhT, input_malloc);
vv[117] = offsetof(qhT, qhull_command);
vv[118] = offsetof(qhT, qhull_commandsiz2);
vv[119] = offsetof(qhT, rbox_command);
vv[120] = offsetof(qhT, qhull_options);
vv[121] = offsetof(qhT, qhull_optionlen);
vv[122] = offsetof(qhT, qhull_optionsiz);
vv[123] = offsetof(qhT, qhull_optionsiz2);
vv[124] = offsetof(qhT, run_id);
vv[125] = offsetof(qhT, VERTEXneighbors);
vv[126] = offsetof(qhT, ZEROcentrum);
vv[127] = offsetof(qhT, upper_threshold);
vv[128] = offsetof(qhT, lower_threshold);
vv[129] = offsetof(qhT, upper_bound);
vv[130] = offsetof(qhT, lower_bound);
vv[131] = offsetof(qhT, ANGLEround);
vv[132] = offsetof(qhT, centrum_radius);
vv[133] = offsetof(qhT, cos_max);
vv[134] = offsetof(qhT, DISTround);
vv[135] = offsetof(qhT, MAXabs_coord);
vv[136] = offsetof(qhT, MAXlastcoord);
vv[137] = offsetof(qhT, MAXoutside);
vv[138] = offsetof(qhT, MAXsumcoord);
vv[139] = offsetof(qhT, MAXwidth);
vv[140] = offsetof(qhT, MINdenom_1);
vv[141] = offsetof(qhT, MINdenom);
vv[142] = offsetof(qhT, MINdenom_1_2);
vv[143] = offsetof(qhT, MINdenom_2);
vv[144] = offsetof(qhT, MINlastcoord);
vv[145] = offsetof(qhT, NEARzero);
vv[146] = offsetof(qhT, NEARinside);
vv[147] = offsetof(qhT, ONEmerge);
vv[148] = offsetof(qhT, outside_err);
vv[149] = offsetof(qhT, WIDEfacet);
vv[150] = offsetof(qhT, NARROWhull);
vv[151] = offsetof(qhT, qhull);
vv[152] = offsetof(qhT, errexit);
vv[153] = offsetof(qhT, jmpXtra);
vv[154] = offsetof(qhT, restartexit);
vv[155] = offsetof(qhT, jmpXtra2);
vv[156] = offsetof(qhT, fin);
vv[157] = offsetof(qhT, fout);
vv[158] = offsetof(qhT, ferr);
vv[159] = offsetof(qhT, interior_point);
vv[160] = offsetof(qhT, normal_size);
vv[161] = offsetof(qhT, center_size);
vv[162] = offsetof(qhT, TEMPsize);
vv[163] = offsetof(qhT, facet_list);
vv[164] = offsetof(qhT, facet_tail);
vv[165] = offsetof(qhT, facet_next);
vv[166] = offsetof(qhT, newfacet_list);
vv[167] = offsetof(qhT, visible_list);
vv[168] = offsetof(qhT, num_visible);
vv[169] = offsetof(qhT, tracefacet_id);
vv[170] = offsetof(qhT, tracefacet);
vv[171] = offsetof(qhT, traceridge_id);
vv[172] = offsetof(qhT, traceridge);
vv[173] = offsetof(qhT, tracevertex_id);
vv[174] = offsetof(qhT, tracevertex);
vv[175] = offsetof(qhT, vertex_list);
vv[176] = offsetof(qhT, vertex_tail);
vv[177] = offsetof(qhT, newvertex_list);
vv[178] = offsetof(qhT, num_facets);
vv[179] = offsetof(qhT, num_vertices);
vv[180] = offsetof(qhT, num_outside);
vv[181] = offsetof(qhT, num_good);
vv[182] = offsetof(qhT, facet_id);
vv[183] = offsetof(qhT, ridge_id);
vv[184] = offsetof(qhT, vertex_id);
vv[185] = offsetof(qhT, first_newfacet);
vv[186] = offsetof(qhT, hulltime);
vv[187] = offsetof(qhT, ALLOWrestart);
vv[188] = offsetof(qhT, build_cnt);
vv[189] = offsetof(qhT, CENTERtype);
vv[190] = offsetof(qhT, furthest_id);
vv[191] = offsetof(qhT, last_errcode);
vv[192] = offsetof(qhT, GOODclosest);
vv[193] = offsetof(qhT, coplanar_apex);
vv[194] = offsetof(qhT, hasAreaVolume);
vv[195] = offsetof(qhT, hasTriangulation);
vv[196] = offsetof(qhT, isRenameVertex);
vv[197] = offsetof(qhT, JOGGLEmax);
vv[198] = offsetof(qhT, maxoutdone);
vv[199] = offsetof(qhT, max_outside);
vv[200] = offsetof(qhT, max_vertex);
vv[201] = offsetof(qhT, min_vertex);
vv[202] = offsetof(qhT, NEWfacets);
vv[203] = offsetof(qhT, NEWtentative);
vv[204] = offsetof(qhT, findbestnew);
vv[205] = offsetof(qhT, findbest_notsharp);
vv[206] = offsetof(qhT, NOerrexit);
vv[207] = offsetof(qhT, PRINTcradius);
vv[208] = offsetof(qhT, PRINTradius);
vv[209] = offsetof(qhT, POSTmerging);
vv[210] = offsetof(qhT, printoutvar);
vv[211] = offsetof(qhT, printoutnum);
vv[212] = offsetof(qhT, repart_facetid);
vv[213] = offsetof(qhT, retry_addpoint);
vv[214] = offsetof(qhT, QHULLfinished);
vv[215] = offsetof(qhT, totarea);
vv[216] = offsetof(qhT, totvol);
vv[217] = offsetof(qhT, visit_id);
vv[218] = offsetof(qhT, vertex_visit);
vv[219] = offsetof(qhT, WAScoplanar);
vv[220] = offsetof(qhT, ZEROall_ok);
vv[221] = offsetof(qhT, facet_mergeset);
vv[222] = offsetof(qhT, degen_mergeset);
vv[223] = offsetof(qhT, vertex_mergeset);
vv[224] = offsetof(qhT, hash_table);
vv[225] = offsetof(qhT, other_points);
vv[226] = offsetof(qhT, del_vertices);
vv[227] = offsetof(qhT, gm_matrix);
vv[228] = offsetof(qhT, gm_row);
vv[229] = offsetof(qhT, line);
vv[230] = offsetof(qhT, maxline);
vv[231] = offsetof(qhT, half_space);
vv[232] = offsetof(qhT, temp_malloc);
vv[233] = offsetof(qhT, ERREXITcalled);
vv[234] = offsetof(qhT, firstcentrum);
vv[235] = offsetof(qhT, old_randomdist);
vv[236] = offsetof(qhT, coplanarfacetset);
vv[237] = offsetof(qhT, last_low);
vv[238] = offsetof(qhT, last_high);
vv[239] = offsetof(qhT, last_newhigh);
vv[240] = offsetof(qhT, lastcpu);
vv[241] = offsetof(qhT, lastfacets);
vv[242] = offsetof(qhT, lastmerges);
vv[243] = offsetof(qhT, lastplanes);
vv[244] = offsetof(qhT, lastdist);
vv[245] = offsetof(qhT, lastreport);
vv[246] = offsetof(qhT, mergereport);
vv[247] = offsetof(qhT, old_tempstack);
vv[248] = offsetof(qhT, ridgeoutnum);
vv[249] = offsetof(qhT, last_random);
vv[250] = offsetof(qhT, rbox_errexit);
vv[251] = offsetof(qhT, jmpXtra3);
vv[252] = offsetof(qhT, rbox_isinteger);
vv[253] = offsetof(qhT, rbox_out_offset);
vv[254] = offsetof(qhT, cpp_object);
vv[255] = offsetof(qhT, qhmem);
vv[256] = offsetof(qhT, qhstat);
}

extern void jl_qhull_facetT_offsets(size_t *vv) {
vv[0] = offsetof(facetT, furthestdist);
vv[1] = offsetof(facetT, maxoutside);
vv[2] = offsetof(facetT, offset);
vv[3] = offsetof(facetT, normal);
vv[4] = offsetof(facetT, f);
vv[5] = offsetof(facetT, f);
vv[6] = offsetof(facetT, f);
vv[7] = offsetof(facetT, f);
vv[8] = offsetof(facetT, f);
vv[9] = offsetof(facetT, f);
vv[10] = offsetof(facetT, center);
vv[11] = offsetof(facetT, previous);
vv[12] = offsetof(facetT, next);
vv[13] = offsetof(facetT, vertices);
vv[14] = offsetof(facetT, ridges);
vv[15] = offsetof(facetT, neighbors);
vv[16] = offsetof(facetT, outsideset);
vv[17] = offsetof(facetT, coplanarset);
vv[18] = offsetof(facetT, visitid);
vv[19] = offsetof(facetT, id);
vv[20] = offsetof(facetT, id) + sizeof(unsigned int);
vv[21] = offsetof(facetT, id) + sizeof(unsigned int);
vv[22] = offsetof(facetT, id) + sizeof(unsigned int);
vv[23] = offsetof(facetT, id) + sizeof(unsigned int);
vv[24] = offsetof(facetT, id) + sizeof(unsigned int);
vv[25] = offsetof(facetT, id) + sizeof(unsigned int);
vv[26] = offsetof(facetT, id) + sizeof(unsigned int);
vv[27] = offsetof(facetT, id) + sizeof(unsigned int);
vv[28] = offsetof(facetT, id) + sizeof(unsigned int);
vv[29] = offsetof(facetT, id) + sizeof(unsigned int);
vv[30] = offsetof(facetT, id) + sizeof(unsigned int);
vv[31] = offsetof(facetT, id) + sizeof(unsigned int);
vv[32] = offsetof(facetT, id) + sizeof(unsigned int);
vv[33] = offsetof(facetT, id) + sizeof(unsigned int);
vv[34] = offsetof(facetT, id) + sizeof(unsigned int);
vv[35] = offsetof(facetT, id) + sizeof(unsigned int);
vv[36] = offsetof(facetT, id) + sizeof(unsigned int);
vv[37] = offsetof(facetT, id) + sizeof(unsigned int);
vv[38] = offsetof(facetT, id) + sizeof(unsigned int);
vv[39] = offsetof(facetT, id) + sizeof(unsigned int);
vv[40] = offsetof(facetT, id) + sizeof(unsigned int);
vv[41] = offsetof(facetT, id) + sizeof(unsigned int);
vv[42] = offsetof(facetT, id) + sizeof(unsigned int);
vv[43] = offsetof(facetT, id) + sizeof(unsigned int);
}

extern void jl_qhull_vertexT_offsets(size_t *vv) {
  vv[0] = offsetof(vertexT, next);
  vv[1] = offsetof(vertexT, previous);
  vv[2] = offsetof(vertexT, point);
  vv[3] = offsetof(vertexT, neighbors);
  vv[4] = offsetof(vertexT, id);
  vv[5] = offsetof(vertexT, visitid);
  vv[6] = offsetof(vertexT, visitid) +sizeof(unsigned int);
  vv[7] = offsetof(vertexT, visitid) +sizeof(unsigned int);
  vv[8] = offsetof(vertexT, visitid) +sizeof(unsigned int);
  vv[9] = offsetof(vertexT, visitid) +sizeof(unsigned int);
  vv[10] = offsetof(vertexT, visitid) +sizeof(unsigned int);
  vv[11] = offsetof(vertexT, visitid) +sizeof(unsigned int);
}

extern void jl_qhull_setT_offsets(size_t *vv) {
  vv[0] = offsetof(setT, maxsize);
  vv[1] = offsetof(setT, e);
}
  
